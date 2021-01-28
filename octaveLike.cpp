#include <octaveLike.h>

std::vector<double> linspace(double a, double b, int n) {
    std::vector<double> array;
    if (n <= 0) return array;
    if (n == 1)
        array.push_back(b); // array.push_back((a + b) * .5);
    else if (n > 1) {
        double step = (b - a) / (n - 1);
        int count = 0;
        while (count < n) {
            array.push_back(a + count*step);
            ++count;
        }
    }
    return array;
}

std::vector<float> linspace(float a, float b, int n) {
    std::vector<float> array;
    if (n <= 0) return array;
    if (n == 1)
        array.push_back(b); // array.push_back((a + b) * .5f);
    else if (n > 1) {
        float step = (b - a) / (n - 1);
        int count = 0;
        while (count < n) {
            array.push_back(a + count*step);
            ++count;
        }
    }
    return array;
}

cv::Mat linspaceMat(double a, double b, int n)
{
    cv::Mat array;
    if (n <= 0) return array;
    if (n > 0) array = cv::Mat(cv::Size(n, 1), CV_64F);
    if (n == 1)
        array.at<double>(0, 0) = b;// array.at<double>(0, 0) = (a + b) * .5;
    else if (n > 1) {
        double step = (b - a) / (n - 1);
        int count = 0;
        while (count < n) {
            array.at<double>(0, count) = a + count * step;
            ++count;
        }
    }
    return array;
}

cv::Mat linspaceMat(float a, float b, int n)
{
    cv::Mat array;
    if (n <= 0) return array;
    if (n > 0) array = cv::Mat(cv::Size(n, 1), CV_32F);
    if (n == 1)
        array.at<double>(0, 0) = b; // array.at<float>(0, 0) = (a + b) * .5f;
    else if (n > 1) {
        float step = (b - a) / (n - 1);
        int count = 0;
        while (count < n) {
            array.at<float>(0, count) = a + count * step;
            ++count;
        }
    }
    return array;
}

void meshgrid(float xa, float xb, int nx, float ya, float yb, int ny, cv::Mat & X, cv::Mat & Y)
{
    if (nx <= 0 || ny <= 0) {
        std::cerr << "Warning: meshgrid() got nx <= 0 or ny <= 0. Check it.\n";
        X.release();
        Y.release();
        return;
    }
    X.create(ny, nx, CV_32F);
    Y.create(ny, nx, CV_32F);
    float dx = nx > 1 ? (xb - xa) / (nx - 1) : 0.0f;
    float dy = ny > 1 ? (yb - ya) / (ny - 1) : 0.0f;
    for (int i = 0; i < ny; i++) {
        for (int j = 0; j < nx; j++)
        {
            X.at<float>(i, j) = xa + j * dx;
            Y.at<float>(i, j) = ya + i * dy;
        }
    }
}

void meshgrid(double xa, double xb, int nx, double ya, double yb, int ny, cv::Mat & X, cv::Mat & Y)
{
    if (nx <= 0 || ny <= 0) {
        std::cerr << "Warning: meshgrid() got nx <= 0 or ny <= 0. Check it.\n";
        X.release();
        Y.release();
        return;
    }
    X.create(ny, nx, CV_64F);
    Y.create(ny, nx, CV_64F);
    double dx = nx > 1 ? (xb - xa) / (nx - 1) : 0.0;
    double dy = ny > 1 ? (yb - ya) / (ny - 1) : 0.0;
    for (int i = 0; i < ny; i++) {
        for (int j = 0; j < nx; j++)
        {
            X.at<double>(i, j) = xa + j * dx;
            Y.at<double>(i, j) = ya + i * dy;
        }
    }
}

