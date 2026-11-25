import asyncio
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from interior_nav import InteriorNav


class InteriorNavTests(unittest.IsolatedAsyncioTestCase):
    def gate(self, acceptance):
        action = NS(ActionServer=Mock(), CancelResponse=NS(ACCEPT=True), GoalResponse=NS(ACCEPT=True, REJECT=False))
        status = NS(GoalStatus=NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6))
        motion = NS(NavigateToPose=NS(Result=object), navigation=Mock(), succeeded=4, frame='map')
        motion.navigation.send_goal_async.return_value = acceptance
        with patch.dict('sys.modules', {'rclpy.action': action, 'action_msgs.msg': status}):
            return InteriorNav(Mock(), motion, lambda: True)

    def request(self, x=1., y=1.):
        return NS(pose=NS(header=NS(frame_id='map'), pose=NS(position=NS(x=x, y=y))))

    async def test_handoff_drains_a_goal_still_awaiting_acceptance(self):
        loop = asyncio.get_running_loop()
        acceptance = loop.create_future()
        completion = loop.create_future()
        gate = self.gate(acceptance)
        outer = Mock(request=self.request(), is_cancel_requested=False)
        inner = Mock(accepted=True)
        inner.get_result_async.return_value = completion
        self.assertFalse(gate.accept_goal(outer.request))
        gate.enable((0., 4., 0., 4.))
        self.assertFalse(gate.accept_goal(self.request(x=5.)))
        running = asyncio.create_task(gate.execute(outer))
        await asyncio.sleep(0)
        self.assertFalse(gate.stop())
        acceptance.set_result(inner)
        await asyncio.sleep(0)
        inner.cancel_goal_async.assert_called_once()
        self.assertFalse(gate.stop())
        completion.set_result(NS(status=5, result='cancelled'))
        self.assertEqual(await running, 'cancelled')
        self.assertTrue(gate.stop())
        self.assertFalse(gate.accept_goal(outer.request))
        outer.abort.assert_called_once()

    async def test_goal_accepted_before_handoff_is_never_forwarded_after_handoff(self):
        gate = self.gate(None)
        outer = Mock(request=self.request(), is_cancel_requested=False)
        gate.enable((0., 4., 0., 4.))
        self.assertTrue(gate.accept_goal(outer.request))
        self.assertTrue(gate.stop())
        await gate.execute(outer)
        gate.motion.navigation.send_goal_async.assert_not_called()
        outer.abort.assert_called_once()
