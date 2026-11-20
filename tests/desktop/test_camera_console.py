import json
from pathlib import Path
from tools.camera_console import qr_snapshot
from tools.qr_station_work import publish_preview


def test_qr_preview_freshness_and_closed_camera_clears_image(tmp_path):
    import numpy as np
    publish_preview(tmp_path/'qr_camera', np.zeros((40, 60, 3), dtype=np.uint8), 1)
    metadata = json.loads((tmp_path/'qr_camera/state.json').read_text())
    assert qr_snapshot(tmp_path, metadata['acquired_unix_s']+.2)['valid']
    assert not qr_snapshot(tmp_path, metadata['acquired_unix_s']+2)['valid']
    publish_preview(tmp_path/'qr_camera')
    assert not qr_snapshot(tmp_path)['valid']
    assert not (tmp_path/'qr_camera/frame.jpg').exists()


def test_missing_qr_owner_does_not_produce_a_frame(tmp_path):
    assert qr_snapshot(tmp_path)['valid'] is False


def test_viewer_releases_its_capture_and_does_not_take_route_camera(tmp_path, monkeypatch):
    from unittest.mock import Mock
    import tools.camera_console as module
    worker = Mock()
    monkeypatch.setattr(module, 'QRStationWork', Mock(return_value=worker))
    monkeypatch.setattr(module.subprocess, 'run', Mock(return_value=Mock(returncode=1)))
    (tmp_path/'status.json').write_text(json.dumps(dict(state='READY')))
    preview = module.QRPreview(tmp_path, dict(camera='/dev/v4l/by-id/test'))
    try:
        preview.view()
        worker.prewarm.assert_called_once()
        preview.release()
        worker.close.assert_called_once()
        (tmp_path/'status.json').write_text(json.dumps(dict(state='SCANNING')))
        preview.view()
        assert preview.worker is None
        assert module.QRStationWork.call_count == 1
    finally:
        preview.close()
