"""Forward the rust_base explorer's goals only during interior exploration."""
import threading
import time

EXPLORER_ACTION = '/team7/v1/interior_navigate_to_pose'


class InteriorNav:
    def __init__(self, node, motion, ready, xy_tree=None):
        from action_msgs.msg import GoalStatus
        from rclpy.action import ActionServer, CancelResponse, GoalResponse
        from rclpy.task import Future

        self.node = node
        self.motion = motion
        self.ready = ready
        self.new_future = lambda: Future(executor=node.executor)
        self.xy_tree = str(xy_tree) if xy_tree is not None else ''
        self.accept, self.reject = GoalResponse.ACCEPT, GoalResponse.REJECT
        self.accept_cancel = CancelResponse.ACCEPT
        self.terminal = {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED}
        self.lock = threading.RLock()
        self.enabled = False
        self.bounds = None
        self.outer = self.inner = None
        self.pending = self.finished = None
        self.cancel_sent = False
        self.pause_since = None
        self.error = None
        self.server = ActionServer(node, motion.NavigateToPose, EXPLORER_ACTION, self.execute,
                                   goal_callback=self.accept_goal, cancel_callback=self.cancel_goal)

    def accept_goal(self, request):
        with self.lock:
            if not (self.enabled and self.ready() and self.pending is None and self.error is None):
                return self.reject
            x0, x1, y0, y1 = self.bounds
            p = request.pose.pose.position
            if request.pose.header.frame_id != self.motion.frame or not (x0 < p.x < x1 and y0 < p.y < y1):
                return self.reject
            if request.behavior_tree not in ('', 'xy_only'):
                return self.reject
            return self.accept

    def cancel_goal(self, handle):
        with self.lock:
            if handle is self.outer:
                self.cancel_forwarded()
        return self.accept_cancel

    def cancel_forwarded(self):
        if self.inner is not None and not self.cancel_sent:
            try:
                self.inner.cancel_goal_async()
                self.cancel_sent = True
            except Exception as error:
                self.error = f'Could not cancel the explorer goal: {error}'

    def enable(self, bounds):
        with self.lock:
            if self.error is not None:
                raise RuntimeError(self.error)
            if self.pause_since is None or (self.outer is None and self.pending is None):
                self.bounds = bounds
                self.enabled = True
                self.pause_since = None

    def pause(self):
        with self.lock:
            self.enabled = False
            if self.outer is None and self.pending is None:
                return True
            if self.pause_since is None:
                self.pause_since = time.monotonic()
            self.cancel_forwarded()
            return False

    def stop(self):
        with self.lock:
            drained = self.pause()
            if self.error:
                raise RuntimeError(self.error)
            if not drained and time.monotonic() - self.pause_since > 8.0:
                raise RuntimeError('Nav2 did not finish cancelling the rust_base explorer goal')
            return drained

    async def execute(self, outer):
        result = self.motion.NavigateToPose.Result()
        previous = None
        with self.lock:
            # A handoff can occur after acceptance, before the callback is scheduled.
            if outer.is_cancel_requested:
                outer.canceled()
                return result
            if self.accept_goal(outer.request) != self.accept:
                outer.abort()
                return result
            if self.outer is not None:
                self.pending = outer
                previous = self.finished
                self.cancel_forwarded()
        if previous is not None:
            await previous
        with self.lock:
            if self.pending is outer:
                self.pending = None
            if outer.is_cancel_requested:
                outer.canceled()
                return result
            if self.outer is not None or self.accept_goal(outer.request) != self.accept:
                outer.abort()
                return result
            self.outer = outer
            self.finished = self.new_future()
            self.cancel_sent = False

        try:
            request = outer.request
            if request.behavior_tree == 'xy_only':
                request.behavior_tree = self.xy_tree
            inner = await self.motion.navigation.send_goal_async(request)
            if not inner.accepted:
                outer.abort()
                return result
            with self.lock:
                self.inner = inner
                if not self.enabled or self.pending is not None or outer.is_cancel_requested:
                    self.cancel_forwarded()
            response = await inner.get_result_async()
            if response.status not in self.terminal:
                raise RuntimeError(f'Nav2 returned a nonterminal explorer goal status {response.status}')
            if outer.is_cancel_requested:
                outer.canceled()
            elif response.status == self.motion.succeeded:
                outer.succeed()
            else:
                outer.abort()
            return response.result
        except Exception as error:
            with self.lock:
                self.error = f'Explorer goal forwarding failed: {error}'
            self.node.get_logger().error(self.error)
            if outer.is_active:
                outer.abort()
            return result
        finally:
            with self.lock:
                self.outer = self.inner = None
                self.finished.set_result(None)
                self.finished = None

    def close(self):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            if self.pause():
                return
            time.sleep(.05)
        self.node.get_logger().error('Explorer goal cancellation was not confirmed before shutdown')
