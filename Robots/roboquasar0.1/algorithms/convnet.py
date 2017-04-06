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
        self.learning_rate = 0.01
        self.epochs = 20
        self.keep_prob = 0.8

        self.saver = None
        self.save_path = "./edge_classification"

        self.frames = None # will turn into np.array after preprocess
        self.labels = None # will turn into np.array after preprocess
        self.weights = None
        self.frame_shape = (7,7,3)
        self.test_frames = None

        self.trained = trained

        # preprocess frames
        self.preprocess(frames_and_labels)

        # train network if not trained
        if not self.trained:
            self.run_network(True)

    def run(self, frames):
        self.test_frames = np.float32(frames)
        print(self.test_frames.shape)
        print(self.test_frames.dtype)

    def preprocess(self, frames_and_labels):
        """
        Normalizes the frame through with respect to each channel
        
        :param frames_and_labels: list with frames and labels
        :return: list which can easily be put into tf.placeholder object
        """
        #shuffle the frames
        np.random.seed(1)
        shuffled_frames = np.array(frames_and_labels)
        np.random.shuffle(shuffled_frames)
        shuffled_frames = shuffled_frames.tolist()

        # normalize the frame
        frames = []
        labels = []

        for frame_and_label in shuffled_frames:
            frame = frame_and_label[0]
            label = [frame_and_label[1]]

            frames.append(frame)
            labels.append(label)

        self.frames = np.float32(frames)
        self.labels = np.float32(labels)

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
        # init = tf.global_variables_initializer()

        tf.reset_default_graph()

        x = tf.placeholder(tf.float32, shape=[None] + list(self.frame_shape), name="input")
        y = tf.placeholder(tf.float32, shape=[None, 1] , name="y")
        keep_prob = tf.placeholder(tf.float32, name="keep_prob")

        # Convolution
        output = self.conv2d(x, 64, (3, 3), (1, 1))
        # output = self.conv2d(output, 32, (3, 3), (1, 1))
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

        init = tf.global_variables_initializer()

        if train:
            with tf.Session() as sess:
                sess.run(init)
                print(self.epochs)
                for epoch in range(self.epochs):
                    # for frame, label in zip(self.frames, self.labels):
                    #     optimizer.run(feed_dict={input:frame, label:label, keep_prob:self.keep_prob})
                    sess.run(optimizer, feed_dict={x:self.frames,
                                                   y:self.labels,
                                                   keep_prob:self.keep_prob,
                                                   })
                    print("Epoch: %s Error: %s" % (epoch, sess.run(cost, feed_dict={x:self.frames,
                                                                                   y:self.labels,
                                                                                   keep_prob:self.keep_prob,
                                                                                  })))
                # Save Model
                self.saver = tf.train.Saver()
                self.saver.save(sess, self.save_path)

            self.trained = True

        else:
            loaded_graph = tf.Graph()
            with tf.Session(graph=loaded_graph) as sess:
                # Load model
                loader = tf.train.import_meta_graph(self.save_path + '.meta')
                loader.restore(sess, self.save_path)
                print()

                # load tensors
                loaded_x = loaded_graph.get_tensor_by_name('input:0')
                loaded_keep_prob = loaded_graph.get_tensor_by_name('keep_prob:0')
                loaded_output = loaded_graph.get_tensor_by_name('output:0')

                value = sess.run(loaded_output, feed_dict={loaded_x: self.test_frames,
                                                           loaded_keep_prob: 1.0})
                return value