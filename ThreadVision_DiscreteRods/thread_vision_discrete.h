#ifndef _thread_vision_discrete_h
#define _thread_vision_discrete_h

#include "threadhypoth_vision_discrete.h"
#include "thread_vision_utils.h"
#include "../DiscreteRods/trajectory_recorder.h"
//#include "../DiscreteRods/thread_discrete.h"
#include <string.h>
#include <map>
#include <stack>
#include <queue>

#define CENTER_IM_IND 0
#define CORRESPONDING_PTS_ERROR_THRESH 4.0
#define TANGENT_ERROR_THRESH 1000.0
#define DIST_FOR_SCORE_CHECK 70.0
#define SCORE_OUT_OF_VIEW 1.0
#define SCORE_THRESH_SET_TO_CONST 0.5  /*if this score is below this, just set it to zero*/

#define IM_VALUE_NOT_SEEN 255
#define IM_VALUE_CHECKED_FOR_BEGIN 200
#define IM_VALUE_USED_IN_THREAD 127

#define ENERGY_DISTANCE_CONST 0
//#define INIT_LENGTH_THREAD_EACH_PIECE 86.9/24.0
//#define MAX_LENGTH_THREAD 87.0   //in mm
//#define TOTAL_LENGTH_INIT -1.0

#define NUM_START_PTS_TO_INIT 1
//#define NUM_TANS_TO_INIT 1 //currently this doesn't do anything, hard coded for 1...

#define DISPLAY_ORIG_BASE "orig cam"
#define DISPLAY_CANNY_BASE "canny cam"
#define CLOSE_DISTANCE_COEFF _rest_length

#define SAVED_IMAGE_BASE "./stereo_test/stereo_test"



USING_PART_OF_NAMESPACE_EIGEN


class Thread_Hypoth;

struct polyline_draw_params {
    Point* pts;
    int size;
    Scalar color;
};

struct glline_draw_params {
    Point3f* vertices;
    int size;
    float color[3];
};

/*
struct twist_and_score {
double twist_angle;
double score;

twist_and_score(const double angle, const double sco) :
twist_angle(angle), score(sco) {}

twist_and_score(const twist_and_score& in) :
twist_angle(in.twist_angle), score(in.score) {}


twist_and_score(){}
};
*/



//helper structs for precomputing distances to canny detected pixels
struct location_and_distance {
    /* Row and col are the location of thread, dist is the distance */
    int row;
    int col;
    int dist;
    location_and_distance* next;

    location_and_distance(const int rowIn, const int colIn, const int distIn) :
    row(rowIn), col(colIn), dist(distIn), next(NULL) {}

    location_and_distance() :
    next(NULL){}

    ~location_and_distance();
};

struct location_and_distance_for_queue {
    //rowCheck and colCheck are the row and col of the new location to add to map
    int rowCheck, colCheck;
    location_and_distance ld;

    location_and_distance_for_queue(const int rowIn, const int colIn, const location_and_distance ldIn) :
    rowCheck(rowIn), colCheck(colIn), ld(ldIn) {}

    location_and_distance_for_queue(){}
};

struct Line_Segment {
    int row1, col1;
    int row2, col2;
};

//Other Structs
struct start_data {
    Point3f pt;
    Vector3d tangent;
};


class Thread_Vision
{
public:
    Thread_Vision();
    Thread_Vision(char* im_base);
    ~Thread_Vision();
    void clear_thread_hypoths();

    /* Main algorithm */
    void initThreadSearch();

    bool findStartPoints();

    bool runThreadSearch();
    void runThreadSearch_nextIms();

    bool generateNextSetOfHypoths();

    bool shouldUseEnergyDistance();

    vector<double>* findTwist(Thread_Hypoth *thread);

    bool isDone();

    vector<thread_hypoth_pair>* nearbyPairsOfThreadHypoths();
    Thread_Hypoth* mergeThreads(Thread_Hypoth* thread1, Thread_Hypoth* thread2);

    void add_possible_next_hypoths(vector<Thread_Hypoth*>& current_thread_hypoths);
    void sort_hypoths(vector<Thread_Hypoth*>& current_thread_hypoths);

    /* Initialize the thread search */
    void updateCanny();
    bool findNextStartPoint(vector<corresponding_pts>& pts, Point3f& initPt);
    bool findNextStartPoint(vector<corresponding_pts>& pts, Point2i& initPtCenterIm);
    bool findTangent(corresponding_pts& start, Vector3d& init_tangent, vector<tangent_and_score>& tangent);
    bool find_next_tan_visual(vector<tangent_and_score>& tangents);
    bool findCorrespondingPointsOtherIms(vector<corresponding_pts>& pts, Point2i initPt, int camWithPt);

    /* During optimizations */
    double scoreProjection3dPoint(const Point3f& pt3d, double* scores=NULL);
    double scoreProjection3dPointAndTanget(const Vector3d& startpt3d, const Vector3d& tan, double* scores=NULL);
    double score2dPoint(const Point2f& pt, int camNum);
    void vectorsForPixel(Point2i& pt, int camNum, vector<Vector2d>& vectors);

    bool isEndPiece(const Point3f pt);
    bool isEndPiece(const int camNum, const Point2i pt);

    /* Canny Scores */
    void precomputeDistanceScores();
    void precomputeSegments();
    int keyForHashMap(int camNum, int row, int col){return col+cols[camNum]*row;}

    map<int,location_and_distance> _cannyDistanceScores[NUMCAMS];
    map<int,vector<Line_Segment*>*> _cannySegments[NUMCAMS];

    const int num_pieces ();

    /* Stereo on clicks */
    void initializeOnClicks();
    void clickOnPoints(Point2i* clickPoints);
    void clickOnPoints(Point3f& clickPoint);

    /* Initialize optimization */
    //void setInitPtFromClicks(){clickOnPoints(_initPtSaved);};
    //void setInitPt(Point3f& init){_initPtSaved = init;};
    //void setEndPt(Point3f& end){_endPtSaved = end;};
    //void setInitTan(Vector3d& init){_initTanSaved = init.normalized();};

    void addStartData(Point3f& pt, Vector3d& tan){
        start_data tmp = {pt, tan.normalized()};
        _start_data.push_back(tmp);
    }

    void clearStartData(){
        _start_data.clear();
    }


    void set_max_length(double max){_max_length_thread = max;}

    /* Random helpers */
    void gray_to_canny();
    void addThreadPointsToDebug(const Scalar& color);
    void addThreadPointsToDebugImages(const Scalar& color);
    void addThreadPointsToDebugImages(const Scalar& color, Thread* thread);
    void add_debug_points_to_ims();
    void display();
    void clear_display(){ //gl_display_for_debug.resize(0);
        for (int i=0; i < NUMCAMS; i++)
        {
            display_for_debug[i].resize(0);
        }
        /*
// Fix memory leak.
// Reference:
// map<int,location_and_distance> _cannyDistanceScores[NUMCAMS];
// map<int,vector<Line_Segment*>*> _cannySegments[NUMCAMS];
        map<int,location_and_distance>::iterator it1;
        for (it1 = _cannyDistanceScores.begin(); it1 != _cannyDistanceScores.end(); it1++)
        {
            delete (*it).second;
        }
        map<int,vector<Line_Segment*>*>::iterator it2;
        for (it2 = _cannySegments->begin(); it2 != _cannySegments->end(); it2++)
        {
            vector<Line_Segment*> v = *((*it2).second);
            for (int j = v.size()-1; j >= 0; j--)
            {
                delete v[j];
                v.pop_back();
            }
        }
        */
    };

    void saveImages(const char* image_save_base, int im_num);
    vector<polyline_draw_params> display_for_debug[NUMCAMS];
    vector<glline_draw_params> gl_display_for_debug;

    vector < vector <Point2i> > to_reproj_clicks[NUMCAMS];
    void set_reproj_fix_canny(const char* filename);
    bool reproj_points_fix_canny;
    void reproj_points_for_canny();


    bool _visual_only;

    Thread* flip_to_hypoth(int hypoth_ind);
    Thread* curr_thread();
    void next_hypoth();
    void prev_hypoth();

    void write_hypoths_to_file(char* filename);


    Capture* _captures[NUMCAMS];
    string _names[NUMCAMS];
    string _orig_display_names[NUMCAMS];
    string _canny_display_names[NUMCAMS];

    Mat* _frames;
    Mat* _cannyIms;
    Mat _cannyIms_display[NUMCAMS];
    Mat* _cannyAngs;
    ThreeCam* _cams;

    int rows[NUMCAMS];
    int cols[NUMCAMS];

    double _max_length_thread;
    //Point3f _initPtSaved;
    //Point3f _endPtSaved;
    //Vector3d _initTanSaved;

    vector<start_data> _start_data;
    vector< vector<Thread_Hypoth*> > _thread_hypoths;
    vector<Thread_Hypoth*>* best_thread_hypoths;
    int curr_hypoth_ind;


    bool hasInit;
    int stepNumber;



    //retrieving thread data
    void get_thread_data(vector<Vector3d>& points, vector<double>& twist_angles);
    void get_thread_data(vector<Vector3d>& points, vector<Matrix3d>& material_frames);
    void get_thread_data(vector<Vector3d>& points, vector<double>& twist_angles, vector<Matrix3d>& material_frames);
    const Matrix3d& start_rot(void);
    const Matrix3d& end_rot(void);
    const Matrix3d& end_bishop(void);
    const double end_angle(void);
    const double angle_at_ind(const int i);

    const Vector3d& start_pos(void);
    const Vector3d& end_pos(void);

    const Vector3d& start_edge(void);
    const Vector3d& end_edge(void);
};



#endif
