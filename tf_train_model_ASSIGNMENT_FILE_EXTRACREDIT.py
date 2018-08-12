import tensorflow as tf
#import numpy as np
import os
from tensorflow.examples.tutorials.mnist import input_data
import matplotlib.pyplot as plt


class build_train:
    def __init__(self):
        self.rootPath = os.path.abspath(os.path.join(os.getcwd(), os.pardir))   # DO NOT EDIT
        self.save_dir = self.rootPath + '/tf_model'                             # DO NOT EDIT


    def build_train_network(self, network):
        print "Lets do dis"
        ############### MNIST DATA #########################################
        mnist = input_data.read_data_sets("MNIST_data/", one_hot=True)      # DO NOT EDIT
        ############### END OF MNIST DATA ##################################

        ############### CONSTRUCT NEURAL NETWORK MODEL HERE ################

        # MODEL
        # INPUT MUST BE 784 array in order be able to train on MNIST
        # INPUT PLACEHOLDERS MUST BE NAME AS name='ph_x' AND name='ph_y_'
        '''
        Follow following format for defining placeholders:
        x = tf.placeholder(data_type, array_shape, name='ph_x')
        y_ = tf.placeholder(data_type, array_shape, name='ph_y_')
        '''


        # OUTPUT VECTOR y MUST BE LENGTH 10, EACH OUTPUT NEURON CORRESPONDS TO A DIGIT 0-9


        x = tf.placeholder(tf.float32, [None, 784], name='ph_x')
        y_ = tf.placeholder(tf.float32, [None, 10], name='ph_y_')


        W1 = tf.Variable(tf.zeros([784, 10]), name ='W1')
        b1 = tf.Variable(tf.zeros([10]), name='b1')

        #y = tf.nn.softmax(tf.matmul(x, W1) + b1, name='op_y')
        y = tf.matmul(x,W1) + b1

        W_conv1 = weight_variable([5, 5, 1, 32])
        b_conv1 = bias_variable([32])
        x_image = tf.reshape(x, [-1,28,28,1])
        h_conv1 = tf.nn.relu(conv2d(x_image, W_conv1) + b_conv1)
        h_pool1 = max_pool_2x2(h_conv1)

        W_conv2 = weight_variable([5, 5, 32, 64])
        b_conv2 = bias_variable([64])
        h_conv2 = tf.nn.relu(conv2d(h_pool1, W_conv2) + b_conv2)
        h_pool2 = max_pool_2x2(h_conv2)

        W_fc1 = weight_variable([7 * 7 * 64, 1024])
        b_fc1 = bias_variable([1024])

        h_pool2_flat = tf.reshape(h_pool2, [-1, 7*7*64])
        h_fc1 = tf.nn.relu(tf.matmul(h_pool2_flat, W_fc1) + b_fc1)

        keep_prob = tf.placeholder(tf.float32)
        h_fc1_drop = tf.nn.dropout(h_fc1, keep_prob)

        W_fc2 = weight_variable([1024, 10])
        b_fc2 = bias_variable([10])
        y_conv = tf.matmul(h_fc1_drop, W_fc2) + b_fc2




        # LOSS FUNCTION, PREDICTION FUNCTION, ACCURACY FUNCTIONS
        # MAKE SURE ACCURCY FUNCTION IS NAMED ---name='op_accuracy'----
        '''
        EXAMPLE OF NAMING ACCURACY FUNCTION:
        accuracy = tf.reduce_mean(tf.cast(prediction, tf.float32), name='op_accuracy')
        '''
        cross_entropy = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(labels=y_, logits=y_conv))
        train_step = tf.train.AdamOptimizer(1e-4).minimize(cross_entropy)
        correct_prediction = tf.equal(tf.argmax(y_conv,1), tf.argmax(y_,1))
        accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))

        
        #print(accuracy.eval(feed_dict={x: mnist.test.images, y_: mnist.test.labels}))

        ############# END OF NEURAL NETWORK MODEL ##########################

        ############# CONSTRUCT TRAINING FUNCTION ##########################

        # TRAINING FUNCTION SHOULD USE YOUR LOSS FUNCTION TO OPTIMIZE THE MODEL PARAMETERS
        #train_step = tf.train.GradientDescentOptimizer(0.5).minimize(cross_entropy, name='op_train')

        ############## YOUR TRAINING FUNCTION GOES HERE ###############################################################
        ############## YOUR TRAINING FUNCTION GOES HERE ###############################################################
        ############## YOUR TRAINING FUNCTION GOES HERE ###############################################################
        ############## YOUR TRAINING FUNCTION GOES HERE ###############################################################

        ############# END OF TRAINING FUNCTION #############################


        ############# CONSTRUCT TRAINING SESSION ###########################
        saver = tf.train.Saver()                                            # DO NOT EDIT
        sess = tf.InteractiveSession()                                      # DO NOT EDIT
        sess.run(tf.global_variables_initializer())                         # DO NOT EDIT

        ############## YOUR TRAINING LOOP CODE GOES HERE #############################################################
        ############## YOUR TRAINING LOOP CODE GOES HERE #############################################################
        ############## YOUR TRAINING LOOP CODE GOES HERE #############################################################
        ############## YOUR TRAINING LOOP CODE GOES HERE #############################################################
        ############## YOUR TRAINING LOOP CODE GOES HERE #############################################################
        ############## YOUR TRAINING LOOP CODE GOES HERE #############################################################

        train_eval = []
        test_eval = []

        time = []

        for i in range(20000): #should be 20,000
            batch = mnist.train.next_batch(50)
            if i%100 == 0:

                train_accuracy = accuracy.eval(feed_dict={x:batch[0], y_: batch[1], keep_prob: 1.0})
                train_eval.append(train_accuracy)

                test_accuracy = accuracy.eval(feed_dict={x: mnist.test.images, y_: mnist.test.labels, keep_prob: 1.0})
                test_eval.append(test_accuracy)

                time.append(i)

            print("step %d, training accuracy %g"%(i, train_accuracy))
            print("step %d, testing accuracy %g"%(i, test_accuracy))
            train_step.run(feed_dict={x: batch[0], y_: batch[1], keep_prob: 0.5})


        ############# END OF TRAINING SESSION ##############################

        ############# SAVE MODEL ###########################################

        saver.save(sess, save_path=self.save_dir, global_step=network)      # DO NOT EDIT
        print('Model Saved')                                                # DO NOT EDIT
        sess.close()                                                        # DO NOT EDIT
        ############# END OF SAVE MODEL ####################################
        ############# OUTPUT ACCURACY PLOT ################################


        #add a legend
        #plt.plot(time, train_eval,'b', time, val_eval,'r', time, test_eval,'m')
        plt.plot(time, train_eval, 'b', time, test_eval, 'g')
        #plt.legend()
        plt.xlabel('Iterations')
        plt.ylabel('Accuracy')
        plt.title('Training Accuracy Evaluation')
        plt.show()

        ############# END OF ACCURACY PLOT ################################

#Weight n Bias init methods
def weight_variable(shape):
  initial = tf.truncated_normal(shape, stddev=0.1)
  return tf.Variable(initial)

def bias_variable(shape):
  initial = tf.constant(0.1, shape=shape)
  return tf.Variable(initial)

#Convolution n Pooling methods
def conv2d(x, W):
  return tf.nn.conv2d(x, W, strides=[1, 1, 1, 1], padding='SAME')

def max_pool_2x2(x):
  return tf.nn.max_pool(x, ksize=[1, 2, 2, 1],
                          strides=[1, 2, 2, 1], padding='SAME')
