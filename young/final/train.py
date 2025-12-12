import os
from anomalib.data import Folder
from anomalib.models import Patchcore
# 최신 anomalib (v2.x) 경로: anomalib.deploy에서 ExportType을 가져옵니다.
from anomalib.deploy import ExportType
# 💡 anomalib.engine에서 Engine을 가져옵니다.
from anomalib.engine import Engine
from pytorch_lightning.callbacks import ModelCheckpoint


def train():
    # ----------------------------
    # 1. 데이터셋 경로 설정
    # ----------------------------
    dataset_root = "/home/young/final_ws/datasets/cube"
    export_root_dir = "weights/openvino"

    if not os.path.exists(dataset_root):
        print("❌ 오류: 데이터셋 폴더가 없습니다.")
        return

    print("=========================================")
    print("  [Step 1] PatchCore 학습 시작")
    print("  데이터 위치:", dataset_root)
    print("=========================================")

    # ----------------------------
    # 2. Folder 기반 데이터셋 (최신 Anomalib v2.x 사양)
    #    - 모든 경로 인자(val_dir, test_dir, abnormal_dir) 제거
    #    - test_split_ratio=0.0 설정으로 모든 test 데이터를 검증에 사용하도록 강제
    # ----------------------------
    datamodule = Folder(
        name="cube",
        root=dataset_root,
        normal_dir="train/good",
        train_batch_size=4,
        eval_batch_size=4,
        num_workers=4,
        
        # 💡 [핵심] 테스트 분할 비율을 0.0으로 설정하여 모든 test 데이터를 검증(Validation)에 사용
        test_split_ratio=0.0, 
    )

    # ----------------------------
    # 3. PatchCore 모델 설정
    # ----------------------------
    model = Patchcore(
        backbone="wide_resnet50_2",
        pre_trained=True,
        coreset_sampling_ratio=0.01,
    )

    # ----------------------------
    # 4. 체크포인트 저장 설정 (타이밍 문제 해결 위해 every_n_epochs=1 설정)
    # ----------------------------
    checkpoint = ModelCheckpoint(
        dirpath="results/checkpoints",
        filename="patchcore",
        save_last=True,
        save_top_k=1,
        monitor="image_AUROC",
        mode="max",
        
        # 💡 [핵심] 매 에포크마다 실행되도록 설정하여 AUROC 로깅 타이밍 문제 해결 시도
        every_n_epochs=1, 
    )

    # ----------------------------
    # 5. Engine 초기화 (max_epochs를 최소 2로 설정하여 AUROC 로깅 시간 확보)
    # ----------------------------
    engine = Engine(
        # 이전 오류와 무관하게 CPU로 설정 (GPU 사용 시 accelerator="gpu"로 변경 가능)
        accelerator="cpu",     
        max_epochs=2,         # 💡 에포크를 최소 2로 설정하여 AUROC 계산 시간 확보
        default_root_dir="results",
        callbacks=[checkpoint],
        log_every_n_steps=1,
    )

    # ----------------------------
    # 6. 학습 실행
    # ----------------------------
    print("🚀 PatchCore 학습을 시작합니다...")
    engine.fit(model=model, datamodule=datamodule)

    # ----------------------------
    # 7. Export (OpenVINO)
    # ----------------------------
    print("\n📦 학습된 모델을 OpenVINO로 변환합니다...")

    os.makedirs(export_root_dir, exist_ok=True)

    engine.export(
        model=model,
        export_type=ExportType.OPENVINO,
        export_root=export_root_dir
    )

    print("=========================================")
    print("  🎉 학습 및 Export 완료!")
    print(f"  결과 모델: {export_root_dir}/model.xml")
    print("=========================================")


if __name__ == "__main__":
    train()