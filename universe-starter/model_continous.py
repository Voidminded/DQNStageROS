import numpy as np
import tensorflow as tf
import tensorflow.contrib.rnn as rnn

def normalized_columns_initializer(std=1.0):
    def _initializer(shape, dtype=None, partition_info=None):
        out = np.random.randn(*shape).astype(np.float32)
        out *= std / np.sqrt(np.square(out).sum(axis=0, keepdims=True))
        return tf.constant(out)
    return _initializer

def flatten(x):
    return tf.reshape(x, [-1, np.prod(x.get_shape().as_list()[1:])])

def conv2d(x, num_filters, name, filter_size=(3, 3), stride=(1, 1), pad="SAME", dtype=tf.float32, collections=None):
    with tf.variable_scope(name):
        stride_shape = [1, stride[0], stride[1], 1]
        filter_shape = [filter_size[0], filter_size[1], int(x.get_shape()[3]), num_filters]

        # there are "num input feature maps * filter height * filter width"
        # inputs to each hidden unit
        fan_in = np.prod(filter_shape[:3])
        # each unit in the lower layer receives a gradient from:
        # "num output feature maps * filter height * filter width" /
        #   pooling size
        fan_out = np.prod(filter_shape[:2]) * num_filters
        # initialize weights with random weights
        w_bound = np.sqrt(6. / (fan_in + fan_out))

        w = tf.get_variable("W", filter_shape, dtype, tf.random_uniform_initializer(-w_bound, w_bound),
                            collections=collections)
        b = tf.get_variable("b", [1, 1, 1, num_filters], initializer=tf.constant_initializer(0.0),
                            collections=collections)
        return tf.nn.conv2d(x, w, stride_shape, pad) + b

def linear(x, size, name, initializer=None, bias_init=0):
    w = tf.get_variable(name + "/w", [x.get_shape()[1], size], initializer=initializer)
    b = tf.get_variable(name + "/b", [size], initializer=tf.constant_initializer(bias_init))
    return tf.matmul(x, w) + b

def categorical_sample(logits, d):
    value = tf.squeeze(tf.multinomial(logits - tf.reduce_max(logits, [1], keep_dims=True), 1), [1])
    return tf.one_hot(value, d)

from IPython.core.debugger import Tracer
class LSTMPolicy(object):
    def __init__(self, ob_space, ac_space):
        self.x = x = tf.placeholder(tf.float32, [None] + list(ob_space))

        x = tf.nn.elu(conv2d(x, 32, "l1", [3, 3], pad="VALID"))
        self.debug_image = x
        for i in range(1,5):
          x = tf.nn.elu(conv2d(x, 32, "l{}".format(i + 1), [3, 3], [2, 2]))
        # introduce a "fake" batch dimension of 1 after flatten so that we can do LSTM over time dim
        x = tf.expand_dims(flatten(x), [0])

        size = 512
        lstm = rnn.BasicLSTMCell(size, state_is_tuple=True)
        self.state_size = lstm.state_size
        step_size = tf.shape(self.x)[:1]

        c_init = np.zeros((1, lstm.state_size.c), np.float32)
        h_init = np.zeros((1, lstm.state_size.h), np.float32)
        self.state_init = [c_init, h_init]
        c_in = tf.placeholder(tf.float32, [1, lstm.state_size.c])
        h_in = tf.placeholder(tf.float32, [1, lstm.state_size.h])
        self.state_in = [c_in, h_in]

        state_in = rnn.LSTMStateTuple(c_in, h_in)
        lstm_outputs, lstm_state = tf.nn.dynamic_rnn(
            lstm, x, initial_state=state_in, sequence_length=step_size,
            time_major=False)
        lstm_c, lstm_h = lstm_state
        x = tf.reshape(lstm_outputs, [-1, size])

        # self.mu = tf.contrib.layers.fully_connected(
        #         inputs=x,
        #         num_outputs=len(ac_space.spaces),
        #         activation_fn=tf.tanh,
        #         weights_initializer=tf.zeros_initializer)
        self.mu = tf.layers.dense( 
                inputs=x,
                units=len(ac_space.spaces),
                activation=tf.tanh,
                kernel_initializer=None,
                name="mu")
        self.mu = tf.squeeze(self.mu)

        # self.sigma = tf.contrib.layers.fully_connected(
        #         inputs=x,
        #         num_outputs=len(ac_space.spaces), 
        #         activation_fn=tf.tanh,
        #         weights_initializer=tf.zeros_initializer)
        self.sigma = tf.layers.dense( 
                inputs=x,
                units=len(ac_space.spaces),
                activation=tf.tanh,
                kernel_initializer=None,
                name="sigma")
        self.sigma = tf.squeeze(self.sigma)
        self.sigma = tf.nn.softplus(self.sigma) + 1e-5
        self.normal_dist = tf.contrib.distributions.Normal(self.mu, self.sigma)
        self.sample = tf.squeeze(self.normal_dist.sample([1]))
        self.sample = tf.clip_by_value(self.sample, -1.0, 1.0)
        # check the action form tanh
        # self.action = tf.clip_by_value(self.action, env.action_space.low[0], env.action_space.high[0])

        self.vf = tf.reshape(linear(x, 1, "value", normalized_columns_initializer(1.0)), [-1])
        self.state_out = [lstm_c[:1, :], lstm_h[:1, :]]
        self.var_list = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, tf.get_variable_scope().name)
        # Tracer()()

    def get_initial_features(self):
        return self.state_init

    def act(self, ob, c, h):
        sess = tf.get_default_session()
        return sess.run([self.sample, self.vf] + self.state_out,
                        {self.x: [ob], self.state_in[0]: c, self.state_in[1]: h})

    def value(self, ob, c, h):
        sess = tf.get_default_session()
        return sess.run(self.vf, {self.x: [ob], self.state_in[0]: c, self.state_in[1]: h})[0]
