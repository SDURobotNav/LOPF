#include<ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>

using namespace cv;
using namespace cv::ml;
using namespace std;
static void help()
{
    cout<< "\n--------------------------------------------------------------------------" << endl
        << "This program shows Support Vector Machines for Non-Linearly Separable Data. " << endl
        << "--------------------------------------------------------------------------"   << endl
        << endl;
}
int main (int* argc, char** argv)
{
    const int max_train_samples = 300;
    Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setC(0.1);
    svm->setKernel(SVM::LINEAR);
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER,(int)1e7,1e-6));
    Mat train_data(max_train_samples, 145, CV_64F);
    Mat labels(max_train_samples,1, CV_32F);

    svm->train(train_data, ROW_SAMPLE,labels);


}