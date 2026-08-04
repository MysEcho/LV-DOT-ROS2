/*
    FILE: dbscan.h
    ------------------
    helper class header for dbscan
*/
#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>

using namespace std;
namespace onboardDetector{
    // plain constants (previously #defines, which clashed with rclcpp enum members)
    constexpr int UNCLASSIFIED = -1;
    constexpr int CORE_POINT = 1;
    constexpr int BORDER_POINT = 2;
    constexpr int NOISE = -2;
    constexpr int SUCCESS = 0;
    constexpr int FAILURE = -3;

    typedef struct Point_
    {
        float x, y, z;  // X, Y, Z position
        int clusterID;  // clustered ID
    }Point;

    class DBSCAN {
    public:    
        DBSCAN(unsigned int minPts, float eps, vector<Point> points){
            m_minPoints = minPts;
            m_epsilon = eps;
            m_points = points;
            m_pointSize = points.size();
        }
        ~DBSCAN(){}

        int run();
        vector<int> calculateCluster(Point point);
        int expandCluster(Point point, int clusterID);
        inline double calculateDistance(const Point& pointCore, const Point& pointTarget);

        int getTotalPointSize() {return m_pointSize;}
        int getMinimumClusterSize() {return m_minPoints;}
        int getEpsilonSize() {return m_epsilon;}
        
    public:
        vector<Point> m_points;
        
    private:    
        unsigned int m_pointSize;
        unsigned int m_minPoints;
        float m_epsilon;
    };
}
#endif // DBSCAN_H
