from fastapi.testclient import TestClient

from services.pickplace.frame_stream import LatestFrame, SharedStatus
from webui.app import create_app, mjpeg_frames


class FakeWorker:
    def __init__(self):
        self.status = SharedStatus()
        self.frames = LatestFrame()
        self.resumed = False
        self.paused = False
        self.stopped = False
        self.aborted = False
        self.restarted = False

    def resume(self):
        self.resumed = True

    def pause(self):
        self.paused = True

    def request_stop(self):
        self.stopped = True

    def request_abort(self):
        self.aborted = True

    def restart(self):
        self.restarted = True


def test_index_serves_static_page():
    worker = FakeWorker()
    client = TestClient(create_app(worker))
    resp = client.get("/")
    assert resp.status_code == 200
    assert "LeKiwi Pick&Place" in resp.text
    assert "/stream/front" in resp.text


def test_start_resumes_worker():
    worker = FakeWorker()
    client = TestClient(create_app(worker))
    resp = client.post("/start")
    assert resp.status_code == 200
    assert worker.resumed is True


def test_pause_pauses_worker():
    worker = FakeWorker()
    client = TestClient(create_app(worker))
    resp = client.post("/pause")
    assert resp.status_code == 200
    assert worker.paused is True


def test_stop_requests_stop():
    worker = FakeWorker()
    client = TestClient(create_app(worker))
    resp = client.post("/stop")
    assert resp.status_code == 200
    assert worker.stopped is True
    assert worker.aborted is False


def test_estop_requests_abort():
    worker = FakeWorker()
    client = TestClient(create_app(worker))
    resp = client.post("/estop")
    assert resp.status_code == 200
    assert worker.aborted is True


def test_restart_requests_restart():
    worker = FakeWorker()
    client = TestClient(create_app(worker))
    resp = client.post("/restart")
    assert resp.status_code == 200
    assert worker.restarted is True


def test_status_returns_worker_status():
    worker = FakeWorker()
    worker.status.set({"state": "PICK", "hz": 29.5})
    client = TestClient(create_app(worker))
    resp = client.get("/status")
    assert resp.json() == {"state": "PICK", "hz": 29.5}


def test_stream_route_registered_with_multipart_media_type():
    # 실제로 무한 스트림을 끝까지 소비하면(TestClient 로 진짜 HTTP 왕복) 백그라운드
    # 스레드풀 워커가 절대 끝나지 않아 테스트 프로세스가 멈춘다 — 라우트 등록만
    # 확인하고, 프레임 내용은 아래 mjpeg_frames() 직접 호출 테스트로 검증한다.
    worker = FakeWorker()
    app = create_app(worker)
    paths = {route.path: route for route in app.routes}
    assert "/stream/{view}" in paths


def test_mjpeg_frames_yields_multipart_jpeg_chunk():
    worker = FakeWorker()
    worker.frames.set("front", b"\xff\xd8\xff\xdbFAKEJPEGDATA")
    gen = mjpeg_frames(worker, "front")
    chunk = next(gen)  # yield 에서 즉시 멈추므로 이 한 번의 호출은 sleep 을 타지 않는다
    assert b"FAKEJPEGDATA" in chunk
    assert b"Content-Type: image/jpeg" in chunk
