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
        task = NS(Future=lambda **_: asyncio.get_running_loop().create_future())
        with patch.dict('sys.modules', {'rclpy.action': action, 'action_msgs.msg': status, 'rclpy.task': task}):
            return InteriorNav(Mock(), motion, lambda: True, '/stack/navigate_forward_xy.xml')

    def request(self, x=1., y=1., behavior_tree=''):
        return NS(pose=NS(header=NS(frame_id='map'), pose=NS(position=NS(x=x, y=y))),
                  behavior_tree=behavior_tree)

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

    async def test_frontier_replacement_cancels_old_goal_and_translates_xy_only(self):
        loop = asyncio.get_running_loop()
        a_acceptance, b_acceptance = loop.create_future(), loop.create_future()
        a_result, b_result = loop.create_future(), loop.create_future()
        a_inner, b_inner = Mock(accepted=True), Mock(accepted=True)
        a_inner.get_result_async.return_value = a_result
        b_inner.get_result_async.return_value = b_result
        gate = self.gate(a_acceptance)
        gate.motion.navigation.send_goal_async.side_effect = [a_acceptance, b_acceptance]
        gate.enable((0., 4., 0., 4.))
        first = Mock(request=self.request(), is_cancel_requested=False)
        first_task = asyncio.create_task(gate.execute(first))
        a_acceptance.set_result(a_inner)
        await asyncio.sleep(0)

        second = Mock(request=self.request(x=2., behavior_tree='xy_only'), is_cancel_requested=False)
        self.assertTrue(gate.accept_goal(second.request))
        second_task = asyncio.create_task(gate.execute(second))
        await asyncio.sleep(0)
        a_inner.cancel_goal_async.assert_called_once()
        self.assertEqual(gate.motion.navigation.send_goal_async.call_count, 1)
        a_result.set_result(NS(status=5, result='cancelled'))
        await first_task
        await asyncio.sleep(0)
        self.assertEqual(gate.motion.navigation.send_goal_async.call_count, 2)
        self.assertEqual(second.request.behavior_tree, '/stack/navigate_forward_xy.xml')
        b_acceptance.set_result(b_inner)
        b_result.set_result(NS(status=4, result='arrived'))
        self.assertEqual(await second_task, 'arrived')
        second.succeed.assert_called_once()
