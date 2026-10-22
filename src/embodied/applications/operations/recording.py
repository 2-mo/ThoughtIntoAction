"""Task + camera recording context. No controller loop, SDK, or automatic success inference."""
from contextlib import ExitStack
import json
import math
import sys

from embodied.core.storage import save_json
from embodied.data.capture.capture import open_cameras
from embodied.data.capture.session import CaptureSession
from embodied.data.episode.annotations import annotate_task_events


class TaskRecording:
    """Use ``with recording as task``; ``recording.path`` retains every attempt.

    cameras is an explicit ID->already-open-source mapping or configured ID list.
    A scene task may reset before collection and evaluate success afterwards.
    Task completion alone leaves outcome unknown. Exceptions retain failed data.
    """
    def __init__(self, robot, name, *, cameras, annotator, confidence, output='outputs/datasets/runtime',
                 scene=None, reset_scene=False, camera_roles=None, anchor=None, video=True):
        if not cameras:
            raise ValueError('Task demonstrations require explicit recording cameras')
        if not isinstance(annotator, str) or not annotator.strip():
            raise ValueError('Provide the demonstration author/annotator')
        if type(confidence) not in (int, float) or not math.isfinite(confidence) or not 0 <= confidence <= 1:
            raise ValueError('Explicit annotation confidence must be in [0, 1]')
        if reset_scene and scene is None:
            raise ValueError('reset_scene requires an explicit simulation task')
        self.robot, self.name, self.cameras = robot, name, cameras
        self.annotator, self.confidence, self.output = annotator, confidence, output
        self.scene, self.reset_scene, self.roles = scene, reset_scene, camera_roles
        self.anchor, self.video = anchor, video
        self.path = self.manifest = self.annotation = self.task = self.capture = None
        self.stack = None

    def __enter__(self):
        if self.stack is not None:
            raise RuntimeError('Task recording contexts cannot be reused')
        self.stack = ExitStack()
        try:
            if self.reset_scene:
                self.scene.reset()
            # Acquire before opening the stream: never mix an earlier owner's samples.
            self.robot._acquire()
            cameras = self.cameras if isinstance(self.cameras, dict) else open_cameras(self.stack, self.cameras)
            self.task = self.robot.task(self.name)
            self.task.__enter__()
            self.capture = CaptureSession(self.robot.run, output=self.output, task=self.name, cameras=cameras,
                demonstration={'schema': 'embodied.demonstration/1', 'kind': 'scripted', 'sources': ['robot-api'],
                               'task_id': self.task.result['task_id'], 'author': self.annotator},
                camera_roles=self.roles, anchor=self.anchor, video=self.video)
            self.capture.__enter__()
            self.path = self.capture.path
            if self.scene is not None:
                save_json(self.path / 'scene.json', {**self.scene.describe(),
                    'recording_start_snapshot': self.scene.simulation.snapshot(), 'reset_before_recording': self.reset_scene})
            return self.task
        except BaseException:
            error = sys.exc_info()
            if self.task is not None and self.task.directory is not None:
                self.__exit__(*error)
            else:
                self.stack.close()
            raise

    def __exit__(self, exc_type, exc, traceback):
        failure, success = None, None
        errors = []
        try:
            try:
                self.task.__exit__(exc_type, exc, traceback)
            except BaseException as error:
                failure = error
            if exc is None and failure is None and self.scene is not None:
                try:
                    success = self.scene.check_success()
                except Exception as error:
                    failure = error
            try:
                if self.capture is not None:
                    self.capture.stop()
                    self.capture.check()
            except Exception as error:
                errors.append(f'Capture: {error}')
                failure = failure or error
            self.path = self.capture.path if self.capture is not None else None
            if self.capture is not None and self.capture.thread is not None and self.capture.thread.is_alive():
                raise RuntimeError('Capture is still finalizing; task metadata stays in ' + str(self.task.directory))
            if self.path is not None:
                self.manifest = json.loads((self.path / 'episode.json').read_text())
                result = self.task.result
                destination = self.path / 'task'
                destination.mkdir(exist_ok=True)
                for name in ('events.jsonl', 'result.json'):
                    source = self.task.directory / name
                    if source.is_file():
                        (destination / name).write_bytes(source.read_bytes())
                outcome = 'failure' if exc is not None or failure is not None or result['status'] == 'failed' or success is False else 'success' if success is True else 'unknown'
                self.manifest.update(task_run_id=result['task_id'], outcome=outcome,
                    task_execution={'task_id': result['task_id'], 'status': result['status'],
                                    'events': 'task/events.jsonl', 'result': 'task/result.json',
                                    'started_monotonic_ns': result['started_monotonic_ns'],
                                    'finished_monotonic_ns': result['finished_monotonic_ns']},
                    scene={'file': 'scene.json', 'success_checked': success is not None, 'success': success} if self.scene else None)
                save_json(self.path / 'episode.json', self.manifest)
                if (destination / 'events.jsonl').is_file() and self.manifest['lifecycle'] != 'recording':
                    try:
                        self.annotation = annotate_task_events(self.path, destination / 'events.jsonl',
                            annotator=self.annotator, confidence=self.confidence, camera_id=self.anchor)
                    except (OSError, ValueError) as error:
                        errors.append(f'Annotation: {error}')
                        failure = failure or error
                if errors:
                    self.manifest['postprocessing_errors'] = errors
                    self.manifest['outcome'] = 'failure'
                    save_json(self.path / 'episode.json', self.manifest)
        finally:
            self.stack.close()
        if exc is None and failure is not None:
            raise RuntimeError(f'Task recording retained at {self.path}: {failure}') from failure
        return False


def record_task(robot, name, **options):
    """Create a TaskRecording; construction performs no I/O or robot operation."""
    return TaskRecording(robot, name, **options)
