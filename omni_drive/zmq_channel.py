import queue
import threading
from collections.abc import Callable

import zmq

RECEIVE_TIMEOUT_MS = 200
SEND_TIMEOUT_MS = 10
RECEIVE_QUEUE_FRAMES = 3
JOIN_TIMEOUT_S = 1.0


class ZmqSubscriber[Rx]:
    def __init__(self, address: str, decoder: Callable[[bytes], Rx]) -> None:
        self._address = address
        self._decoder = decoder
        self._latest: Rx | None = None
        self._count = 0
        self._lock = threading.Lock()
        self._stopping = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"SUB-{address}", daemon=True)
        self._thread.start()

    @property
    def latest(self) -> Rx | None:
        with self._lock:
            return self._latest

    def latest_with_count(self) -> tuple[Rx | None, int]:
        with self._lock:
            return self._latest, self._count

    def close(self) -> None:
        self._stopping.set()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=JOIN_TIMEOUT_S)

    def _run(self) -> None:
        socket = zmq.Context.instance().socket(zmq.SUB)
        socket.setsockopt(zmq.RCVHWM, RECEIVE_QUEUE_FRAMES)
        socket.setsockopt(zmq.RCVTIMEO, RECEIVE_TIMEOUT_MS)
        socket.connect(self._address)
        socket.subscribe(b"")
        try:
            while not self._stopping.is_set():
                try:
                    message = self._decoder(socket.recv())
                except zmq.Again:
                    continue
                except zmq.ZMQError:
                    break
                except Exception as error:
                    print(f"Ignored an unreadable message on {self._address}: {error}")
                    continue
                with self._lock:
                    self._latest = message
                    self._count += 1
        finally:
            socket.close(linger=0)


class ZmqPublisher[Tx]:
    def __init__(self, address: str, encoder: Callable[[Tx], bytes], queue_size: int = 5) -> None:
        self._address = address
        self._encoder = encoder
        self._queue_size = queue_size
        self._outbox: queue.Queue[bytes | None] = queue.Queue(maxsize=queue_size)
        self._thread = threading.Thread(target=self._run, name=f"PUSH-{address}", daemon=True)
        self._thread.start()

    def send(self, message: Tx) -> None:
        self._put_dropping_oldest(self._encoder(message))

    def close(self) -> None:
        if self._thread.is_alive():
            self._put_dropping_oldest(None)
            self._thread.join(timeout=JOIN_TIMEOUT_S)

    def _run(self) -> None:
        socket = zmq.Context.instance().socket(zmq.PUSH)
        socket.setsockopt(zmq.SNDHWM, self._queue_size)
        socket.setsockopt(zmq.SNDTIMEO, SEND_TIMEOUT_MS)
        socket.connect(self._address)
        try:
            while (frame := self._outbox.get()) is not None:
                try:
                    socket.send(frame)
                except zmq.Again:
                    pass
                except zmq.ZMQError as error:
                    print(f"Could not send to {self._address}: {error}")
        finally:
            socket.close(linger=0)

    def _put_dropping_oldest(self, frame: bytes | None) -> None:
        while True:
            try:
                self._outbox.put_nowait(frame)
                return
            except queue.Full:
                try:
                    self._outbox.get_nowait()
                except queue.Empty:
                    pass
