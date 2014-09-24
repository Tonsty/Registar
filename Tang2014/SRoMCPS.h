#include <vector>
#include <Eigen/Dense>


typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Point;
typedef Point SPoint;
typedef Point TPoint;
typedef std::pair<SPoint, TPoint> PointPair;
typedef Scalar Weight;
struct PointPairWithWeight
{
	PointPair ppair;
	Weight w;
};
typedef std::vector<PointPairWithWeight, Eigen::aligned_allocator<PointPairWithWeight> > PointPairWithWeights;
typedef unsigned int ScanIndex; 
typedef std::pair<ScanIndex, ScanIndex> ScanIndexPair;
typedef std::vector<ScanIndexPair> ScanIndexPairs;

class SRoMCPS
{
public:

	SRoMCPS(ScanIndexPairs &_sipairs, std::vector<PointPairWithWeights> &_ppairwwss, int _M);

	void createViewSelectionMatrices();

	void createQ();

	void createQ_R();

	void createQ_RT();

	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> pseudo_inverse(Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> squareMatrix, const Scalar pinvtoler = Eigen::NumTraits<Scalar>::epsilon());

	void solve_RT();

	void solve_R();

	void solve_T();

	ScanIndexPairs sipairs;
	std::vector<PointPairWithWeights> ppairwwss;

	int M;
	Eigen::Matrix<Scalar, 3, Eigen::Dynamic> R;
	Eigen::Matrix<Scalar, Eigen::Dynamic, 1> T;

	int P;
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> C_a;
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> C_b;

	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Q;
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Q_R;
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Q_RT;

	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> W;

	std::vector< Eigen::Matrix<Scalar, 3, 1> > x_mean, y_mean;


};