import rospy


class SubscriberValue:
    def __init__(self, name, data_class, wait=True, queue_size=1, transform=None):  # type: (str, Any, bool, int, Optional[Callable[[Any], Any]]) -> None
        self._subscriber = rospy.Subscriber(name, data_class, callback=self._callback, queue_size=queue_size)
        self._topic = name
        self._wait = wait
        self._transform = transform
        self._value = None
        self._n = 0

    def _callback(self, message):
        self._n += 1
        if self._transform is None:
            self._value = message
        else:
            self._value = self._transform(message)

    def wait(self):
        while self._value is None and not rospy.is_shutdown():
            rospy.loginfo('Waiting for {}...'.format(self._topic))
            rospy.sleep(0.1)
        return self._value

    @property
    def value(self):
        if self._wait:
            self.wait()
        return self._value

    def wait_for_n_messages(self, num_messages):  # type: (int) -> Any
        target = self._n + num_messages
        while self._n < target and not rospy.is_shutdown():
            rospy.loginfo('Waiting for {}... {}/{}'.format(self._topic, target-self._n, self._n))
            rospy.sleep(0.1)
        return self.value