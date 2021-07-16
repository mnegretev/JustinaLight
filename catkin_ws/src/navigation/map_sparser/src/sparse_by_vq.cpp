#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "random_numbers/random_numbers.h"

random_numbers::RandomNumberGenerator rnd;

nav_msgs::OccupancyGrid get_inflated_map(nav_msgs::OccupancyGrid& map, float inflation)
{
    /*
     * WARNING!!! It is assumed that map borders (borders with at least 'inflation' thickness)
     * are occupied or unkwnon. Map must be big enough to fulfill this assumption.
     */
    if(inflation <= 0)
        return map;
    
    nav_msgs::OccupancyGrid newMap = map;
    int n = (int)(inflation / map.info.resolution);
    int lower_limit = n*map.info.width + n;
    int upper_limit = map.data.size() - n*map.info.width - n;
    for(int k = lower_limit; k < upper_limit; k++)
        if(map.data[k] != 0)
            for(int i=-n; i<=n; i++)
                for(int j=-n; j<=n; j++)
                    newMap.data[k + j*map.info.width + i] = map.data[k];

    return newMap;
}

std::vector<geometry_msgs::Point> get_free_points(nav_msgs::OccupancyGrid& map)
{
    std::vector<geometry_msgs::Point> points;
    for(size_t i=0; i < map.data.size(); i++)
    {
        if(map.data[i] != 0)
            continue;
        geometry_msgs::Point p;
        p.x = (i % map.info.width)*map.info.resolution + map.info.origin.position.x;
        p.y = (i / map.info.width)*map.info.resolution + map.info.origin.position.y;
        points.push_back(p);
    }
    return points;
}

std::vector<geometry_msgs::Point> disturb_centroids(std::vector<geometry_msgs::Point> centroids, float epsilon)
{
    std::vector<geometry_msgs::Point> new_centroids;
    for(size_t i=0; i < centroids.size(); i++)
    {
        float a = rnd.uniformReal(-M_PI, M_PI);
        geometry_msgs::Point p1,p2;
        p1.x =  epsilon*cos(a);
        p1.y =  epsilon*sin(a);
        p2.x = -epsilon*cos(a);
        p2.y = -epsilon*sin(a);
        new_centroids.push_back(p1);
        new_centroids.push_back(p2);
    }
    return new_centroids;
}

int get_nearest_centroid_idx(geometry_msgs::Point& p, std::vector<geometry_msgs::Point>& centroids)
{
    float min = INT_MAX;
    int nearest_idx = -1;
    for(size_t i=0; i < centroids.size(); i++)
    {
        float d = sqrt((p.x - centroids[i].x)*(p.x - centroids[i].x) + (p.y - centroids[i].y)*(p.y - centroids[i].y));
        if(d < min)
        {
            min = d;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}

std::vector<geometry_msgs::Point> get_centroids(std::vector<geometry_msgs::Point>& points, int m, float epsilon, float tol)
{
    geometry_msgs::Point c;
    for(size_t i=0; i < points.size(); i++)
    {
        c.x += points[i].x;
        c.y += points[i].y;
    }
    c.x /= points.size();
    c.y /= points.size();
    std::cout << "MapSparser.->First centroid: " << c << std::endl;;
    std::vector<geometry_msgs::Point> centroids;
    centroids.push_back(c);
    while(centroids.size() < m && ros::ok())
    {
        centroids = disturb_centroids(centroids, epsilon);
        std::cout << "MapSparser.->Calculating " << centroids.size() << " centroids" << std::endl;
        float delta = tol + 1;
        while(delta > tol && ros::ok())
        {
            std::vector<geometry_msgs::Point> new_centroids;
            std::vector<int> counters;
            for(size_t i=0; i < centroids.size(); i++)
            {
                new_centroids.push_back(geometry_msgs::Point());
                counters.push_back(0);
            }
            for(size_t i=0; i < points.size(); i++)
            {
                int idx = get_nearest_centroid_idx(points[i], centroids);
                new_centroids[idx].x += points[i].x;
                new_centroids[idx].y += points[i].y;
                counters[idx]++;
            }
            delta = 0;
            for(size_t i=0; i < centroids.size(); i++)
            {
                new_centroids[i].x = counters[i] > 0 ? new_centroids[i].x/counters[i] : 0;
                new_centroids[i].y = counters[i] > 0 ? new_centroids[i].y/counters[i] : 0;
                delta += fabs(new_centroids[i].x - centroids[i].x) + fabs(new_centroids[i].y - centroids[i].y);
            }
            delta /= centroids.size();
            std::cout << "MapSparser.->Current delta: " << delta << std::endl;
            centroids = new_centroids;
        }
    }
    return centroids;
}

int main(int argc, char** argv)
{
    std::cout << "MapSparser.->INITIALIZING MAP SPARSER..." << std::endl;
    ros::init(argc, argv, "map_sparser");
    ros::NodeHandle n;

    int   num_centroids = 4;
    float epsilon       = 0.1;
    float tolerance     = 0.1;
    float inflation_radius = 0.3;
    if(ros::param::has("~centroids"))
        ros::param::get("~centroids", num_centroids);
    if(ros::param::has("~epsilon"))
        ros::param::get("~epsilon", epsilon);
    if(ros::param::has("~tolerance"))
        ros::param::get("~tolerance", tolerance);
    if(ros::param::has("~inflation_radius"))
        ros::param::get("~inflation_radius", inflation_radius);
    
    ros::Rate loop(1);

    std::cout << "MapSparser.->Waiting for static map topic..." << std::endl;
    ros::service::waitForService("/static_map", ros::Duration(1000.0));
    ros::ServiceClient cltGetStaticMap = n.serviceClient<nav_msgs::GetMap>("/static_map");
    nav_msgs::GetMap srvStaticMap;
    cltGetStaticMap.call(srvStaticMap);
    std::cout << "MapSparser.->Static map received." << std::endl;
    std::cout << "MapSparser.->Inflating map by " << inflation_radius << " ..." << std::endl;
    nav_msgs::OccupancyGrid map = get_inflated_map(srvStaticMap.response.map, inflation_radius);
    std::vector<geometry_msgs::Point> points = get_free_points(map);
    std::cout << "MapSparser.->Inflated map with " << points.size() << " free cells" << std::endl;
    std::vector<geometry_msgs::Point> centroids = get_centroids(points, num_centroids, epsilon, tolerance);
    return 0;
}
