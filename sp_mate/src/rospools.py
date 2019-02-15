import rospy

class PublisherPool:
    def __init__(self, Message_Class, topic_suffix, queue_size=10):
        self.pool = dict()
        self.MsgClass = Message_Class
        self.topic_suffix = topic_suffix
        self.queue_size = queue_size

    def get_pub(self, namespace):
        if namespace in self.pool:
            return self.pool[namespace]
        else:
            topic = '{}/{}'.format(namespace, self.topic_suffix)
            pub = rospy.Publisher(topic, self.MsgClass, queue_size=self.queue_size)
            self.pool[namespace] = pub
            return pub

class ServiceProxyPool:
    def __init__(self, Srv_class, srv_suffix):
        self.pool = dict()
        self.MsgClass = Srv_class
        self.srv_suffix = srv_suffix

    def get_pub(self, namespace):
        if namespace in self.pool:
            return self.pool[namespace]
        else:
            topic = '{}/{}'.format(namespace, self.srv_suffix)
            rospy.wait_for_service(topic)
            srv = rospy.ServiceProxy(topic, self.MsgClass)
            self.pool[namespace] = srv
            return srv
