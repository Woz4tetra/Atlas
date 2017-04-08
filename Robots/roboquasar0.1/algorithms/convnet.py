import os
import cv2

import tensorflow as tf
import numpy as np


class NeuralNetwork:
    def __init__(self, frame_shape, good_dir=None, bad_dir=None, trained=False, epochs=20):
        """
        Initializes a simple convolution network for proper edge classification

        :param frames_and_labels: list of [frame, label] elements
        :param frames_shape: shape of the frame
        :param trained: whether the network is trained already
        :param epochs: number of epochs to run the training
        """
        # Hyperparameters
        self.learning_rate = 0.0001
        self.epochs = epochs
        self.keep_prob = 0.9

        self.saver = None
        self.save_path = "./algorithms/trained_networks/edge_classification"

        self.frames = None # will turn into np.array after pre_process
        self.labels = None # will turn into np.array after pre_process

        self.frame_shape = frame_shape
        self.batch_size = 40

        self.frame_label_queue = []

        self.trained = trained
        self.output_value = None

        # pre-process frames
        if not self.trained:
            self.pre_process(good_dir, bad_dir)

        # train network if not trained
        if not self.trained:
            self.train_network()

    def run(self, frames):
        test_frames = np.float32(frames)
        self.run_network(test_frames)

    def normalize(self, frame):
        filter_frame = np.zeros(list(self.frame_shape))

        for c in range(3):
            max_v = np.max(frame[:, :, c])
            min_v = np.min(frame[:, :, c])
            filter_frame[:, :, c] = frame[:, :, c] / 255

        return filter_frame

    def pre_process(self, dir_good, dir_bad):
        frames_and_labels = []

        for entry in os.listdir(dir_good):
            path = "%s/%s" % (dir_good, entry)
            frames_and_labels.append([path, 1])

        for entry in os.listdir(dir_bad):
            path = "%s/%s" % (dir_bad, entry)
            frames_and_labels.append([path, 0])

        np.random.seed(1)
        shuffled = np.array(frames_and_labels)
        np.random.shuffle(shuffled)
        self.frame_label_queue = shuffled.tolist()

    def process(self, frames_and_labels):
        """
        Normalizes the frame through with respect to each channel

        :param frames_and_labels: list with frames and labels
        :return: list which can easily be put into tf.placeholder object
        """
        #shuffle the frames
        self.frames = None
        self.labels = None

        # normalize the frame
        frames = []
        labels = []

        for frame_and_label in frames_and_labels:
            frame = frame_and_label[0]
            label = [frame_and_label[1]]

            frame = cv2.imread(frame)

            if frame.shape == self.frame_shape:
                frame = self.normalize(frame)
                frames.append(frame)
                labels.append(label)

        self.frames = np.float32(frames)
        self.labels = np.float32(labels)

    def conv2d(self, x_tensor, conv_num_outputs, conv_ksize, conv_strides, pool_ksize, pool_strides):
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
        W1 = tf.Variable(tf.truncated_normal(W_shape, stddev=0.01))
        b1 = tf.Variable(tf.truncated_normal([conv_num_outputs], stddev=0.01))

        # apply a convolution
        x = tf.nn.conv2d(x_tensor, W1, strides=conv_strides, padding='SAME')
        x = tf.nn.bias_add(x, b1)
        x = tf.nn.relu(x)

        # define max_strides, ksize_shape
        pool_strides = [1] + list(pool_strides) + [1]
        pool_ksize = [1] + list(pool_ksize) + [1]
        x = tf.nn.max_pool(x, ksize=pool_ksize, strides=pool_strides, padding='SAME')

        return x

    def create_batches(self, batch_size):
        pass

    def fully_connected(self, x_tensor, num_outputs):
        """
        Creates a fully connected neural network layer
        :param x_tensor: input tensor
        :param num_outputs: number of outputs
        :return: output of a fully connected layer
        """
        return tf.contrib.layers.fully_connected(inputs=x_tensor, num_outputs=num_outputs,
                                                 activation_fn=tf.sigmoid)

    def output_layer(self, x_tensor, num_outputs):
        return tf.contrib.layers.fully_connected(inputs=x_tensor, num_outputs=num_outputs,
                                                 activation_fn=None)

    def train_network(self):
        """
        trains a network with preprocessed data
        """
        tf.reset_default_graph()
        tf.set_random_seed(1)

        x = tf.placeholder(tf.float32, shape=[None] + list(self.frame_shape), name="input")
        y = tf.placeholder(tf.float32, shape=[None, 1] , name="y")
        keep_prob = tf.placeholder(tf.float32, name="keep_prob")
        two = tf.constant(2.0)

        test = tf.multiply(x, two, name="test")

        # Convolution
        output = self.conv2d(x, 32, (3, 3), (1, 1), (2,2), (2,2))
        output = self.conv2d(output, 64, (3, 3), (1, 1), (2, 2), (2, 2))

        output = tf.nn.dropout(output, keep_prob)

        output = tf.contrib.layers.flatten(output)

        # Fully Connected Layer
        output = self.fully_connected(output, 20)
        output = self.fully_connected(output, 1)
        output = tf.identity(output, name="output")

        # cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=output, labels=y))

        error = tf.subtract(y, output)
        cost = tf.reduce_mean(tf.square(error))

        optimizer = tf.train.AdamOptimizer(self.learning_rate).minimize(cost)

        # init = tf.global_variables_initializer()

        if not self.trained:
            with tf.Session() as sess:
                sess.run(tf.global_variables_initializer())
                print("Number of epochs: %s" % (self.epochs))
                number_batches = int(len(self.frame_label_queue) / self.batch_size)
                print("Number of batches: %s" % (number_batches))

                batch_init = 0
                batch_end = self.batch_size
                data_size = len(self.frame_label_queue)

                for batch in range(number_batches):
                # for train_frame, train_label in zip(train_frames, train_labels):
                    if data_size - batch_init < self.batch_size:
                        batch_end = data_size

                    if batch_end - batch_init == 0:
                        break

                    print(len(self.frame_label_queue[batch_init:batch_end]))
                    self.process(self.frame_label_queue[batch_init:batch_end])

                    print("----- Batch %s -----" % (batch+1))
                    for epoch in range(self.epochs):

                        sess.run(optimizer, feed_dict={x:self.frames,
                                                       y:self.labels,
                                                       keep_prob:self.keep_prob,
                                                       })

                        print("Epoch: %s Error: %s" % (epoch, sess.run(cost, feed_dict={x:self.frames,
                                                                                        y:self.labels,
                                                                                        keep_prob: self.keep_prob,
                                                                                      })))
                    batch_init += self.batch_size
                    batch_end += self.batch_size
                # Save Model
                self.saver = tf.train.Saver()
                self.saver.save(sess, self.save_path)

            self.trained = True

    def run_network(self, test_frame):

        frames = np.resize(test_frame, tuple([1] + list(self.frame_shape)))
        # tf.reset_default_graph()

        loaded_graph = tf.Graph()
        with tf.Session(graph=loaded_graph) as sess:
            # Load model
            loader = tf.train.import_meta_graph(self.save_path + '.meta')
            loader.restore(sess, self.save_path)

            # load tensors
            loaded_x = loaded_graph.get_tensor_by_name('input:0')
            loaded_keep_prob = loaded_graph.get_tensor_by_name('keep_prob:0')
            loaded_output = loaded_graph.get_tensor_by_name('output:0')

            val = sess.run(loaded_output,
                           feed_dict={loaded_x: frames,
                           loaded_keep_prob: 1.0})

            self.output_value = val[0][0]

    #TODO: Implement retrain function for training on the go
