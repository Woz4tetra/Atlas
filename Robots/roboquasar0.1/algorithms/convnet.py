import tensorflow as tf
import numpy as np

class NeuralNetwork:
    def __init__(self, frames_and_labels, trained=False):
        """
        Initializes a simple convolutional network for proper edge classification
        
        :param keep_prob: probability of keeping data at dropout step
        :param frames: 5x5 frames for the training set
        :param labels: 1/0 labels that say whether correct line or not
        """
        # Hyperparameters
        self.learning_rate = 0.1
        self.epochs = 20
        self.keep_prob = 0.9

        self.saver = None
        self.save_path = "/tmp/model.ckpt"

        self.frames = None # will turn into np.array after preprocess
        self.labels = None # will turn into np.array after preprocess
        self.weights = None
        self.frame_shape = (7,7,3)
        self.test_frame = None

        self.trained = trained

        # preprocess frames
        self.preprocess(frames_and_labels)

        # make sure everything is ok
        # assert(len(self.frames) == len(self.labels))
        print(self.frames.shape)
        print(self.labels.shape)

        # train network if not trained
        if not self.trained:
            self.run_network(True)

    def run(self, frame):
        if not self.trained:
            self.run_network(True)
        else:
            self.test_frame = frame
            self.run_network(False)

    def preprocess(self, frames_and_labels):
        """
        Normalizes the frame through with respect to each channel
        
        :param frames_and_labels: list with frames and labels
        :return: list which can easily be put into tf.placeholder object
        """
        # normalize the frame
        frames = []
        labels = []

        for frame_and_label in frames_and_labels:
            frame = frame_and_label[0]
            label = np.array([frame_and_label[1]])

            frames.append(frame)
            labels.append(label)

        self.frames = np.array(frames)
        self.labels = np.array(labels)

    # def conv2d(self, x_tensor, conv_num_outputs, conv_ksize, conv_strides):
    #     """
    #     Apply convolution then max pooling to x_tensor
    #     :param x_tensor: TensorFlow Tensor
    #     :param conv_num_outputs: Number of outputs for the convolutional layer
    #     :param conv_ksize: kernal size 2-D Tuple for the convolutional layer
    #     :param conv_strides: Stride 2-D Tuple for convolution
    #     :param pool_ksize: kernal size 2-D Tuple for pool
    #     :param pool_strides: Stride 2-D Tuple for pool
    #     : return: A tensor that represents convolution and max pooling of x_tensor
    #     """
    #     # define strides, W, and b
    #     depth_original = x_tensor.get_shape().as_list()[3]
    #     conv_strides = [1] + list(conv_strides) + [1]
    #
    #     W_shape = list(conv_ksize) + [depth_original] + [conv_num_outputs]
    #     W1 = tf.Variable(tf.truncated_normal(W_shape, stddev=0.01), name="w1")
    #     b1 = tf.Variable(tf.truncated_normal([conv_num_outputs], stddev=0.01), name="b1")
    #
    #     # apply a convolution
    #     x = tf.nn.conv2d(x_tensor, W1, strides=conv_strides, padding='SAME')
    #     x = tf.nn.bias_add(x, b1)
    #     x = tf.nn.relu(x)
    #
    #     self.saver = tf.train.Saver({"weight":W1,
    #                                  "bias": b1})
    #
    #     return x
    #
    # def fully_connected(self, x_tensor, num_outputs):
    #     """
    #     Creates a fully connected neural network layer
    #     :param x_tensor: input tensor
    #     :param num_outputs: number of outputs
    #     :return: output of a fully connected layer
    #     """
    #     return tf.contrib.layers.fully_connected(inputs=x_tensor, num_outputs=num_outputs,
    #                                              activation_fn=tf.sigmoid)

    def run_network(self, train):
        """
        trains a network with preprocessed data
        """
        # init = tf.global_variables_initializer()

        x = tf.placeholder(tf.float32, shape=([None] + list(self.frame_shape)), name="input")
        y = tf.placeholder(tf.float32, shape=[None, 1], name="label")
        keep_prob = tf.placeholder(tf.float32, name="keep_prob")

        # Convolution
        conv_num_outputs = 32
        conv_ksize = (3,3)
        conv_strides = (1,1)
        depth_original = x.get_shape().as_list()[3]
        conv_strides = [1] + list(conv_strides) + [1]

        W_shape = list(conv_ksize) + [depth_original] + [conv_num_outputs]
        W1 = tf.Variable(tf.truncated_normal(W_shape, stddev=0.01), name="w1")
        b1 = tf.Variable(tf.truncated_normal([conv_num_outputs], stddev=0.01), name="b1")

        # apply a convolution
        conv = tf.nn.conv2d(x, W1, strides=conv_strides, padding='SAME')
        conv = tf.nn.bias_add(conv, b1)
        output = tf.nn.relu(conv)

        # self.saver = tf.train.Saver({"weight": W1,
        #                              "bias": b1})
        # self.saver = tf.train.Saver()

        output = tf.nn.dropout(output, keep_prob)
        output = tf.contrib.layers.flatten(output)

        # apply a fully connected layer
        output = tf.contrib.layers.fully_connected(inputs=output, num_outputs=1,activation_fn=tf.sigmoid)

        cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=output, labels=y))


        error = tf.subtract(y, output)
        mse = tf.reduce_mean(tf.square(error))

        # optimizer = tf.train.GradientDescentOptimizer(learning_rate=0.1).minimize(mse)
        optimizer = tf.train.AdamOptimizer().minimize(cost)

        init_op = tf.initialize_all_variables()

        if train:
            with tf.Session() as sess:
                # sess.run(init)
                sess.run(init_op)
                for epoch in range(self.epochs):
                    # for frame, label in zip(self.frames, self.labels):
                    #     optimizer.run(feed_dict={input:frame, label:label, keep_prob:self.keep_prob})
                    sess.run(optimizer, feed_dict={x: self.frames.astype(np.float),
                                                   y: self.labels.astype(np.float),
                                                   keep_prob: self.keep_prob
                                                   })
                    print("Epoch: {} Error: {}".format(epoch, sess.run(mse)))

                    self.saver.save(sess, self.save_path)
                print()

            self.trained = True

        else:
            with tf.Session() as sess:
                self.saver.restore(sess, self.save_path)
                value = sess.run(output, feed_dict={x: self.test_frame,
                                                    keep_prob: 1.0,
                                                    })
                return value