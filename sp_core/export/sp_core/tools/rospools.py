import rospy

class PublisherPool:
    """
    Pool of dynamically-created publishers, to send messages to a same topic name in different namespaces
    """
    def __init__(self, Message_Class, topic_suffix, queue_size=10):
        self.pool = dict()
        self.MsgClass = Message_Class
        self.topic_suffix = topic_suffix
        self.queue_size = queue_size

    def get_pub(self, namespace):
        """
        Get the publisher for the given ``<namespace>/<topic_suffix>``
        
        If the publisher doesn't exist, it is created and stored for future reuse.
        
        :param namespace: name of the namespace, without trailing '/'. Ex: ``/cf1``.
        :type namespace: str
        :return: Publisher for the topic
        :rtype: rospy.Publisher

        Example::

            cmd_vel_pool = PublisherPool(Twist, 'cmd_vel')
            cf1_cmd_vel_publisher = cmd_vel_pool.get_pub('cf1')
            cf1_msg = Twist()
            # customise message here

            cf1_cmd_vel_publisher.publish(cf1_msg)

        """
        if namespace in self.pool:
            return self.pool[namespace]
        else:
            topic = '{}/{}'.format(namespace, self.topic_suffix)
            pub = rospy.Publisher(topic, self.MsgClass, queue_size=self.queue_size)
            self.pool[namespace] = pub
            return pub

class ServiceProxyPool:
    """
    Pool of dynamically-created Service proxies.
    """
    def __init__(self, Srv_class, srv_suffix):
        self.pool = dict()
        self.MsgClass = Srv_class
        self.srv_suffix = srv_suffix

    def get_svp(self, namespace):
        """
        Get the service caller function for the service ``<namespace>/<topic_suffix>``
        
        If the service proxy function doesn't exist, it is created and stored for future use
        
        :param namespace: name of the namespace, without trailing '/'. Ex: ``/cf1``
        :type namespace: str
        :return: Service Proxy function for requested name
        :rtype: rospy.ServiceProxy

        Example::

            position_pool = ServiceProxyPool(PositionSrv, 'local_position')
            position_cf1_service = position_pool.get_svp('/cf1')
            current_pos_cf1 = position_cf1_service(service_arg1="value1", ...)
        """
        if namespace in self.pool:
            return self.pool[namespace]
        else:
            topic = '{}/{}'.format(namespace, self.srv_suffix)
            rospy.wait_for_service(topic)
            srv = rospy.ServiceProxy(topic, self.MsgClass)
            self.pool[namespace] = srv
            return srv
