import tensorflow as tf
import os
import numpy as np
# from tensorflow.examples.tutorials.mnist import input_data

class evaluate_model:
    def __init__(self):
        self.rootPath = os.path.abspath(os.path.join(os.getcwd(), os.pardir))   # DO NOT EDIT
        self.save_dir = self.rootPath + '/tf_model'                             # DO NOT EDIT

    def evaluate_model(self,model_version,input):

        # mnist = input_data.read_data_sets("MNIST_data/", one_hot=True)

        filename = self.save_dir + '-' + str(model_version) + '.meta'           # DO NOT EDIT
        filename2 = self.save_dir + '-' + str(model_version)                    # DO NOT EDIT
        print('Opening Model: ' + str(filename))                                # DO NOT EDIT
        saver = tf.train.import_meta_graph(filename)                            # DO NOT EDIT
        sess = tf.InteractiveSession()                                          # DO NOT EDIT

        # INITIALIZE GLOBAL VARIABLES
        sess.run(tf.global_variables_initializer())                             # DO NOT EDIT

        # RESTORE MODEL LOSS OP AND INPUT PLACEHOLDERS
        saver.restore(sess, filename2)                                          # DO NOT EDIT
        graph = tf.get_default_graph()                                          # DO NOT EDIT


        # accuracy = graph.get_tensor_by_name('op_accuracy:0')
        x = graph.get_tensor_by_name('ph_x:0')
        # y_ = graph.get_tensor_by_name('ph_y_:0')
        y = graph.get_tensor_by_name('op_y:0')
        output = tf.argmax(y,1)


        print('Classifying Image...')
        batch_input = np.expand_dims(input, axis=0)
        output = sess.run(output, feed_dict={x: batch_input})
        sess.close()
        # print('type is: ' + type(output).__name__)
        # print(output)
        # print output[0]
        return output[0]


