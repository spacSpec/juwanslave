import os
import glob
import numpy as np
import torch
import torch.nn.functional as F
from PIL import Image
from torchvision import transforms
from torchvision.models import wide_resnet50_2, Wide_ResNet50_2_Weights
from scipy.spatial.distance import mahalanobis
from scipy.linalg import inv
from sklearn.metrics import roc_auc_score

# ----------------------------------------------------
# 1. 설정 및 상수 정의
# ----------------------------------------------------

# 데이터셋의 최상위 경로 (사용자 환경에 맞게 확인 필수)
# 전처리(Alignment)가 완료된 이미지들이 들어있어야 합니다.
DATASET_ROOT = "/home/young/final_ws/datasets/cube" 
TRAIN_GOOD_DIR = os.path.join(DATASET_ROOT, "train", "good")
TEST_DIR = os.path.join(DATASET_ROOT, "test")

# PaDiM 모델 결과 저장 경로
MODEL_OUTPUT_DIR = "padim_weights/cube"
os.makedirs(MODEL_OUTPUT_DIR, exist_ok=True)

# PaDiM 설정
NUM_RANDOM_CHANNELS = 300 # 1500은 메모리가 터질 수 있어 550 정도로 줄이는 경우도 있음 (RAM 충분하면 1500 유지)
COV_IDENTITY_FACTOR = 0.01 
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# [핵심 수정 사항] 전처리 파이프라인
# CenterCrop을 제거하고 (224, 224)로 강제 리사이즈합니다.
# 이미 타이트하게 잘린(Alignment) 이미지가 들어오므로, 잘라내지 않고 찌그러뜨려서라도 전체를 다 봐야 합니다.
TRANSFORM = transforms.Compose([
    transforms.Resize((224, 224), Image.LANCZOS), # 가로세로 224로 강제 고정
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

# ----------------------------------------------------
# 2. ResNet 특징 추출기 및 유틸리티 함수
# ----------------------------------------------------

def get_resnet_backbone():
    """Wide ResNet50-2 모델을 로드하고 특징 추출을 위한 Hook을 설정합니다."""
    # weights 매개변수 사용 (최신 PyTorch 버전 대응)
    model = wide_resnet50_2(weights=Wide_ResNet50_2_Weights.IMAGENET1K_V2)
    
    feature_maps = {}
    def hook_fn(module, input, output, name):
        # Layer 3 특징 맵은 Layer 2 크기에 맞게 업샘플링 (PaDiM 표준)
        if name == 'layer3':
            output = F.interpolate(output, size=(28, 28), mode='bilinear', align_corners=False)
        # Layer 1 특징 맵은 Layer 2 크기에 맞게 다운샘플링 (PaDiM 표준)
        elif name == 'layer1':
             output = F.avg_pool2d(output, kernel_size=2)
             
        feature_maps[name] = output

    # Hook 연결 (Layer 1, 2, 3)
    model.layer1.register_forward_hook(lambda m, i, o: hook_fn(m, i, o, 'layer1'))
    model.layer2.register_forward_hook(lambda m, i, o: hook_fn(m, i, o, 'layer2'))
    model.layer3.register_forward_hook(lambda m, i, o: hook_fn(m, i, o, 'layer3'))
    
    model.fc = torch.nn.Identity()
    model.eval()
    model.to(DEVICE)
    
    return model, feature_maps

def extract_patch_features(model, feature_maps, image_tensor):
    """단일 이미지에 대해 특징을 추출하고 패치 단위로 합쳐서 반환합니다."""
    
    with torch.no_grad():
        _ = model(image_tensor.to(DEVICE))
    
    # Layer 1, 2, 3의 특징 맵을 채널 차원에서 연결: (1, 1792, 28, 28)
    combined_features = torch.cat([feature_maps['layer1'], feature_maps['layer2'], feature_maps['layer3']], dim=1)
    
    # (B, C, H, W) -> (B, H*W, C) 형태로 변환 -> (784, 1792)
    patch_features = combined_features.permute(0, 2, 3, 1).flatten(1, 2).squeeze(0) 
    
    return patch_features.cpu().numpy()

# ----------------------------------------------------
# 3. 학습 단계 (분포 모델링)
# ----------------------------------------------------

def train_padim_model(model, feature_maps):
    """정상 이미지를 사용하여 패치 특징의 평균과 공분산을 계산합니다."""
    
    image_files = glob.glob(os.path.join(TRAIN_GOOD_DIR, '*.png')) + \
                  glob.glob(os.path.join(TRAIN_GOOD_DIR, '*.jpg'))
    
    if not image_files:
        print(f"❌ 오류: '{TRAIN_GOOD_DIR}'에서 이미지를 찾을 수 없습니다.")
        return None, None, None

    print(f"\n[Step 1: 학습] 총 {len(image_files)}개의 정상 이미지로 PaDiM 학습 시작.")
    
    all_patch_features = []
    
    for i, file_path in enumerate(image_files):
        try:
            image = Image.open(file_path).convert('RGB')
            tensor = TRANSFORM(image).unsqueeze(0) # 여기서 Resize((224,224)) 적용됨
            
            patch_features = extract_patch_features(model, feature_maps, tensor)
            all_patch_features.append(patch_features)
            
            print(f"   > 이미지 {i+1}/{len(image_files)} 처리 완료.", end='\r')
        except Exception as e:
            print(f"\n   ⚠️ 이미지 처리 오류: {file_path}, 오류: {e}")
            continue

    if not all_patch_features:
        print("\n❌ 특징 추출 실패. 데이터를 확인하세요.")
        return None, None, None

    # 메모리 주의: 이미지가 많으면 여기서 OOM(Out of Memory) 발생 가능
    all_features_concatenated = np.concatenate(all_patch_features, axis=0)
    total_channels = all_features_concatenated.shape[1] 

    # 1. 랜덤 채널 선택 (차원 축소)
    # NUM_RANDOM_CHANNELS가 전체 채널보다 크면 전체 채널 사용
    n_channels = min(NUM_RANDOM_CHANNELS, total_channels)
    random_channels = np.random.choice(total_channels, n_channels, replace=False)
    final_features = all_features_concatenated[:, random_channels]
    
    print(f"\n✅ 특징 추출 완료. 최종 특징 행렬 형태: {final_features.shape}")

    # 2. 평균 벡터 (μ) 계산
    mean_vector = np.mean(final_features, axis=0)
    
    # 3. 공분산 행렬 (Σ) 계산
    cov_matrix = np.cov(final_features, rowvar=False)
    
    # 4. 정규화 및 역행렬 계산
    identity = np.eye(cov_matrix.shape[0]) * COV_IDENTITY_FACTOR 
    cov_matrix += identity
    inv_cov_matrix = inv(cov_matrix)

    # 5. 저장
    np.save(os.path.join(MODEL_OUTPUT_DIR, 'mean_vector.npy'), mean_vector)
    np.save(os.path.join(MODEL_OUTPUT_DIR, 'inv_cov_matrix.npy'), inv_cov_matrix)
    np.save(os.path.join(MODEL_OUTPUT_DIR, 'random_channels.npy'), random_channels)
    
    print(f"\n🎉 PaDiM 학습 완료! 모델이 '{MODEL_OUTPUT_DIR}'에 저장되었습니다.")
    
    return mean_vector, inv_cov_matrix, random_channels

# ----------------------------------------------------
# 4. 테스트 단계 (평가)
# ----------------------------------------------------

def test_padim_model(model, feature_maps, mean_vector, inv_cov_matrix, random_channels):
    
    all_test_files = []
    all_test_labels = [] # 0: good, 1: anomaly

    if not os.path.exists(TEST_DIR):
        print(f"\n❌ 오류: 테스트 폴더 '{TEST_DIR}'가 없습니다.")
        return

    # test 폴더 구조: test/good, test/defect_type1, ...
    for class_dir in os.listdir(TEST_DIR):
        class_path = os.path.join(TEST_DIR, class_dir)
        if os.path.isdir(class_path):
            # 'good' 폴더면 라벨 0, 그 외(불량)면 라벨 1
            label = 0 if class_dir == 'good' else 1
            files = glob.glob(os.path.join(class_path, '*.png')) + \
                    glob.glob(os.path.join(class_path, '*.jpg'))
            
            all_test_files.extend(files)
            all_test_labels.extend([label] * len(files))

    if not all_test_files:
        print(f"\n❌ 오류: 테스트 이미지를 찾을 수 없습니다.")
        return

    print(f"\n[Step 2: 테스트] 총 {len(all_test_files)}개의 이미지 평가 시작.")
    
    image_anomaly_scores = []
    
    for i, file_path in enumerate(all_test_files):
        try:
            image = Image.open(file_path).convert('RGB')
            tensor = TRANSFORM(image).unsqueeze(0) # Resize((224,224)) 적용
            
            patch_features_full = extract_patch_features(model, feature_maps, tensor)
            patch_features_reduced = patch_features_full[:, random_channels] 
            
            # 마할라노비스 거리 계산 (Scipy cdist 사용하면 더 빠르지만, 가독성을 위해 루프 유지)
            # 대량 처리시에는 cdist(patch_features_reduced, [mean_vector], metric='mahalanobis', VI=inv_cov_matrix) 권장
            mahala_distances = []
            for patch_feat in patch_features_reduced:
                dist = mahalanobis(patch_feat, mean_vector, inv_cov_matrix)
                mahala_distances.append(dist)
            
            # [사용자 로직 반영] 상위 N개 패치 거리 평균 계산
            N = 10 
            sorted_distances = np.sort(mahala_distances)[::-1] # 내림차순 정렬
            top_n_distances = sorted_distances[:min(N, len(sorted_distances))]
            image_score = np.mean(top_n_distances)
            
            image_anomaly_scores.append(image_score)
            
            # 진행 상황 표시
            print(f"   > 처리 중: {i+1}/{len(all_test_files)} | Score: {image_score:.4f}", end='\r')
            
        except Exception as e:
            print(f"\n   ⚠️ 오류: {file_path} - {e}")
            image_anomaly_scores.append(0.0)

    # ROC AUC 평가
    image_anomaly_scores = np.array(image_anomaly_scores)
    all_test_labels = np.array(all_test_labels)

    if len(np.unique(all_test_labels)) > 1:
        image_auc = roc_auc_score(all_test_labels, image_anomaly_scores)
        print(f"\n\n==============================================")
        print(f"✨ Image-level ROC AUC: {image_auc:.4f}")
        print(f"==============================================")
    else:
        print("\n\n⚠️ AUC 계산 불가: 테스트 데이터에 정상 또는 불량 중 하나만 존재합니다.")

# ----------------------------------------------------
# 5. 실행
# ----------------------------------------------------

if __name__ == "__main__":
    
    # 1. 모델 준비
    model, feature_maps = get_resnet_backbone()
    
    # 2. 학습 (정상 데이터 분포 모델링)
    mean_vec, inv_cov_mat, rand_channels = train_padim_model(model, feature_maps)
    
    # 3. 테스트 (이상 탐지 성능 평가)
    if mean_vec is not None:
        test_padim_model(model, feature_maps, mean_vec, inv_cov_mat, rand_channels)