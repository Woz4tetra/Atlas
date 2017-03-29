import tensorflow as tf
import numpy as np

class NeuralNetwork(object):
    def __init__(self, *frames, *labels):
        """
        Initializes a simple convolutional network for proper edge classification
        
        :param keep_prob: probability of keeping data at dropout step
        :param frames: 5x5 frames for the training set
        :param labels: 1/0 labels that say whether correct line or not
        """
        # Hyperparameters
        self.learning_rate = 0.1
        self.epochs = 20
        self.keep_prob = 1.0

        self.saver = None
        self.save_path = "./tmp/model.ckpt"

        self.frames = []
        self.labels = []
        self.weights = None
        self.frame_shape = [7,7,3]
        self.test_frame = None

        self.trained = False

        # preprocess frames
        for frame, label in zip(frames, labels):
            self.preprocess(frame, label)

        # make sure everything is ok
        assert(len(frames) == len(labels))

        # train network if not trained
        if not self.trained:
            self.run_network(True)

    def run(self, frame):
        if not self.trained:
            self.run_network(True)
        else:
            self.test_frame = frame
            self.run_network(False)

    def preprocess(self, frame, label):
        """
        Normalizes the frame through with respect to each channel
        
        :param frame: a 7x7 frame
        :param label: a label (either a 0 or 1)
        :return: list which can easily be put into tf.placeholder object
        """
        # normalize the frame
        filter_frame = np.zeros(list(self.frame_shape)) + 1

        for c in range(3):
            max_v = np.max(frame[:, :, c]) * 1.0
            min_v = np.min(frame[:, :, c]) * 1.0
            filter_frame[:, :, c] = (frame[:, :, c] - min_v) / (max_v - min_v)

        self.frames += filter_frame
        self.labels += label

    def conv2d(self, x_tensor, conv_num_outputs, conv_ksize, conv_strides):
        """
        Apply convolution then max pooling to x_tensor
        :param x_tensor: TensorFlow Tensor
        :param conv_num_outputs: Number of outputs for the convolutional layer
        :param conv_ksize: kernal size 2-D Tuple for the convolutional layer
        :param conv_strides: Stride 2-D Tuple for convolution
        :param pool_ksize: kernal size 2-D Tuple for pool
        :param pool_strides: Stride 2-D Tuple for pool
        : return: A tensor that represents convolution and max pooling of x_tensor
        """
        # define strides, W, and b
        depth_original = x_tensor.get_shape().as_list()[3]
        conv_strides = [1] + list(conv_strides) + [1]

        W_shape = list(conv_ksize) + [depth_original] + [conv_num_outputs]
        W1 = tf.Variable(tf.truncated_normal(W_shape, stddev=0.01), name="w1")
        b1 = tf.Variable(tf.truncated_normal([conv_num_outputs], stddev=0.01), name="b1")

        # apply a convolution
        x = tf.nn.conv2d(x_tensor, W1, strides=conv_strides, padding='SAME')
        x = tf.nn.bias_add(x, b1)
        x = tf.nn.relu(x)

        self.saver = tf.train.Saver({"weight":W1,
                                     "bias": b1})

        return x

    def fully_connected(self, x_tensor, num_outputs):
        """
        Creates a fully connected neural network layer
        :param x_tensor: input tensor
        :param num_outputs: number of outputs
        :return: output of a fully connected layer
        """
        return tf.contrib.layers.fully_connected(inputs=x_tensor, num_outputs=num_outputs,
                                                 activation_fn=tf.sigmoid)

    def run_network(self, train):
        """
        trains a network with preprocessed data
        """
        init_op = tf.global_variables_initializer()

        input = tf.placeholder(tf.float32, shape=[None] + self.frame_shape, name="input")
        label = tf.placeholder(tf.float32, shape=[None, 1], name="label")
        keep_prob = tf.placeholder(tf.float32, name="keep_prob")

        output = self.conv2d(input, 32, (3, 3), (1, 1))
        output = tf.nn.dropout(output, keep_prob)
        output = tf.contrib.layers.flatten(output)

        output = self.fully_connected(output, 1)

        error = tf.subtract(label, output)
        mse = tf.reduce_mean(tf.square(error))

        optimizer = tf.train.GradientDescentOptimizer(mse, self.learning_rate)

        if train:
            with tf.Session() as sess:
                sess.run(init_op)
                for epoch in range(self.epochs):
                    sess.run(optimizer, feed_dict={input:self.frames,
                                                   label:self.labels,
                                                   keep_prob:self.keep_prob,
                                                   })
                    self.saver.save(sess, self.save_path)
                print()

            self.trained = True

        else:
            with tf.Session() as sess:
                self.saver.restore(sess, self.save_path)
                value = sess.run(output, feed_dict={input: self.test_frame,
                                                    keep_prob: 1.0,
                                                    })
                return value