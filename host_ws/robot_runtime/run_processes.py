"""Shut down the processes launched by this V1 run, including orphaned children."""
import ctypes
from dataclasses import dataclass
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


PR_SET_CHILD_SUBREAPER = 36
SHUTDOWN_STAGES = ((signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 3))


class StopRequested(Exception):
    pass


@dataclass(frozen=True)
class ProcessInfo:
    pid: int
    parent: int
    session: int
    zombie: bool
    started: str | None = None


def linux_process(pid):
    try:
        # The executable name may contain spaces and parentheses.
        fields = Path(f'/proc/{pid}/stat').read_text(errors='replace').rpartition(') ')[2].split()
        return ProcessInfo(pid, int(fields[1]), int(fields[3]), fields[0] in ('Z', 'X'), fields[19])
    except (OSError, IndexError, ValueError):
        return None


def snapshot():
    return [entry for path in Path('/proc').iterdir() if path.name.isdigit()
            if (entry := linux_process(int(path.name))) is not None]


def descendants(processes, roots):
    children = {}
    for process in processes:
        children.setdefault(process.parent, []).append(process.pid)
    found = set(roots)
    pending = list(found)
    while pending:
        new = set(children.get(pending.pop(), ())) - found
        found.update(new)
        pending.extend(new)
    return found


def signal_process(process, sig):
    try:
        if process.started is None:
            os.kill(process.pid, sig)
        elif not hasattr(os, 'pidfd_open'):
            current = linux_process(process.pid)
            if current is not None and current.started == process.started:
                os.kill(process.pid, sig)
        else:
            # Bind the signal to a process identity even if its PID is recycled.
            fd = os.pidfd_open(process.pid)
            try:
                current = linux_process(process.pid)
                if current is not None and current.started == process.started:
                    signal.pidfd_send_signal(fd, sig)
            finally:
                os.close(fd)
    except (ProcessLookupError, PermissionError):
        pass


class RunProcesses:
    def __init__(self):
        if sys.platform == 'linux':
            libc = ctypes.CDLL(None, use_errno=True)
            if libc.prctl(PR_SET_CHILD_SUBREAPER, 1, 0, 0, 0) != 0:
                error = ctypes.get_errno()
                raise OSError(error, os.strerror(error))
        self.pid = os.getpid()
        self.jobs = {}
        self.interrupted = None
        self.closed = False
        self.remaining = []

    def request_stop(self, signum, _frame):
        if self.interrupted is None:
            self.interrupted = signum

    def check(self):
        if self.interrupted is not None:
            raise StopRequested

    def start(self, command, **kwargs):
        self.check()
        process = subprocess.Popen(command, start_new_session=True, **kwargs)
        self.jobs[process.pid] = process
        return process

    def run(self, command, *, cwd):
        process = self.start(command, cwd=cwd)
        while process.poll() is None:
            self.check()
            time.sleep(.1)
        self.check()
        if process.returncode != 0:
            raise subprocess.CalledProcessError(process.returncode, command)

    def members(self, session):
        for job in self.jobs.values():
            job.poll()
        processes = snapshot()
        if sys.platform == 'linux':
            for process in processes:
                if process.zombie and process.parent == self.pid and process.pid not in self.jobs:
                    try:
                        os.waitpid(process.pid, os.WNOHANG)
                    except ChildProcessError:
                        pass
        owned = descendants(processes, {self.pid})
        if session is not None:
            roots = {entry.pid for entry in processes if entry.session == session and entry.pid in owned}
            owned = descendants(processes, roots)
        return [entry for entry in processes if not entry.zombie and entry.pid in owned and entry.pid != self.pid]

    def stop(self, session):
        for sig, wait in SHUTDOWN_STAGES:
            deadline = time.monotonic() + wait
            notified = set()
            while True:
                members = self.members(session)
                if not members:
                    return []
                for process in members:
                    if process not in notified:
                        signal_process(process, sig)
                        notified.add(process)
                if time.monotonic() >= deadline:
                    break
                time.sleep(.1)
        return self.members(session)

    def close(self):
        if self.closed:
            return
        self.closed = True
        # The mission was launched last. Give it time to cancel Nav2 before stopping the stack.
        for session in reversed(self.jobs):
            self.stop(session)
        self.remaining = self.stop(None)
        if self.remaining:
            pids = ', '.join(str(process.pid) for process in self.remaining)
            print(f'Could not stop all V1 processes. Remaining PIDs: {pids}', file=sys.stderr, flush=True)
