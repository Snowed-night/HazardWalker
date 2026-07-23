"""防止五套历史感知素材再次被误删或误标为官方成绩。"""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SIMULATION_ROOT = ROOT / "reports" / "perception" / "simulation" / "3d_native"
TEST_RECORD_ROOT = ROOT / "reports" / "perception" / "test_records"
SUITES = (
    "official_simenv_20260710_active_multiview_reobservation",
    "official_simenv_20260710_rgbd_localization",
    "official_simenv_20260710_multi_ball_clutter",
    "official_simenv_20260710_partial_visibility",
    "official_simenv_20260710_extended_red_object_stress",
)
RECLASSIFIED_SUITES = (
    "official_simenv_20260705_red_ball_detection",
    "official_simenv_20260705_partial_visibility",
    "official_simenv_20260705_multi_ball_clutter",
    "official_simenv_20260705_extended_red_object_stress",
    *SUITES,
    "official_simenv_20260715_partial_visibility",
    "official_simenv_20260715_extended_red_object_stress",
)


def _read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def test_historical_suites_keep_visual_and_structured_records():
    for suite in SUITES:
        suite_dir = SIMULATION_ROOT / suite
        assert (suite_dir / "README.md").is_file()
        assert (suite_dir / "summary.json").is_file()
        assert (suite_dir / "cases.csv").is_file()
        assert any((suite_dir / "images").glob("*.png"))


def test_historical_suites_cannot_be_mistaken_for_official_scores():
    for suite in SUITES:
        summary = _read_json(SIMULATION_ROOT / suite / "summary.json")
        testing_record = _read_json(
            TEST_RECORD_ROOT / suite / "testing_record_perception.json"
        )
        for record in (summary, testing_record):
            assert record["evidence_class"] == "historical_internal_regression"
            assert record["official_score_eligible"] is False
            assert record["rerun_required"] is True
            assert record["known_limitations"]


def test_historical_readmes_define_same_directory_reruns():
    for suite in SUITES:
        readme = (SIMULATION_ROOT / suite / "README.md").read_text(encoding="utf-8")
        assert "历史内部回归" in readme
        assert "reruns/YYYYMMDD_<seed>/" in readme


def test_reclassified_stages_keep_provenance_and_local_test_records():
    """目录改名不能丢失溯源和实验目录内测试组表格。"""

    for suite in RECLASSIFIED_SUITES:
        suite_dir = SIMULATION_ROOT / suite
        provenance = _read_json(suite_dir / "provenance.json")
        assert provenance["classification_status"] in {
            "verified", "provenance_uncertain",
        }
        assert (suite_dir / "testing_record_perception.csv").is_file()
        assert (suite_dir / "testing_record_perception.json").is_file()
