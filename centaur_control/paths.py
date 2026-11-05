"""统一管理项目输入和输出路径，不依赖命令执行目录。"""
from pathlib import Path
PACKAGE_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_ROOT.parent
MODEL_PATH = PACKAGE_ROOT / "centaur_unified.xml"
RESULTS_DIR = PACKAGE_ROOT / "analysis_results"
IMPACTS_DIR = RESULTS_DIR / "impacts"
FORMAL_DIR = RESULTS_DIR / "formal"
CHECKS_DIR = RESULTS_DIR / "checks"
BENCHMARK_DIR = RESULTS_DIR / "benchmark"


def analysis_output(output_dir=None, *, formal=True):
    """低分辨率实验不得写入正式结果目录，旧结果保持原位。"""
    directory = Path(output_dir).resolve() if output_dir is not None else FORMAL_DIR if formal else CHECKS_DIR
    if not formal and (directory == FORMAL_DIR.resolve() or FORMAL_DIR.resolve() in directory.parents):
        raise ValueError("低采样/粗扫描只能写入 checks 或其他实验目录，不能覆盖 formal")
    directory.mkdir(parents=True, exist_ok=True)
    return directory
