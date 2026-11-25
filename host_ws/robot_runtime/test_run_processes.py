import signal
import unittest
from unittest.mock import Mock, patch

from run_processes import ProcessInfo, RunProcesses, StopRequested


class ShutdownTests(unittest.TestCase):
    def test_stage_includes_reparented_job_children_and_ignores_unrelated_processes(self):
        runner = RunProcesses()
        leader = 5100
        runner.jobs[leader] = Mock(returncode=0)
        orphan = ProcessInfo(5101, runner.pid, leader, False)
        detached = ProcessInfo(5102, orphan.pid, 5102, False)
        unrelated = ProcessInfo(5103, 1, leader, False)
        with patch('run_processes.snapshot', return_value=[orphan, detached, unrelated]):
            self.assertEqual(runner.members(leader), [orphan, detached])
            self.assertEqual(runner.members(None), [orphan, detached])

    def test_repeated_interrupts_do_not_interrupt_cleanup_and_stubborn_children_escalate(self):
        runner = RunProcesses()
        stack, mission = 5100, 5200
        runner.jobs = {stack: Mock(returncode=0), mission: Mock(returncode=0)}
        worker = ProcessInfo(5201, runner.pid, mission, False)
        calls = []

        def members(session):
            if session == mission and (worker.pid, signal.SIGKILL) not in calls:
                return [worker]
            return []

        def signal_worker(process, sig):
            calls.append((process.pid, sig))
            runner.request_stop(signal.SIGINT, None)

        runner.request_stop(signal.SIGINT, None)
        with self.assertRaises(StopRequested):
            runner.check()
        stages = tuple((sig, 0.0) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL))
        with (patch('run_processes.SHUTDOWN_STAGES', stages),
              patch.object(runner, 'members', side_effect=members) as get_members,
              patch('run_processes.signal_process', side_effect=signal_worker)):
            runner.close()
            runner.close()
        self.assertEqual(calls, [(worker.pid, sig) for sig, _ in stages])
        self.assertEqual([call.args[0] for call in get_members.call_args_list], [mission] * 4 + [stack, None])
        self.assertEqual(runner.remaining, [])
