#ifndef PAIRWISEREGISTRATION_H
#define PAIRWISEREGISTRATION_H

#include <QtCore/QObject>
#include <QtCore/QVariantMap>

#ifndef Q_MOC_RUNC
#include <vector>
#include <Eigen/Dense>
#include "pclbase.h"
#include "registrationdatamanager.h"
#endif

namespace registar
{
	struct Correspondence
	{
		PointType targetPoint;
		PointType sourcePoint;
	};

	typedef std::vector<Correspondence, Eigen::aligned_allocator<Correspondence> > Correspondences;

	struct CorrespondenceIndex
	{
		int targetIndex;
		int sourceIndex;
	};
	typedef std::vector<CorrespondenceIndex, Eigen::aligned_allocator<CorrespondenceIndex> > CorrespondenceIndices;

	struct CorrespondencesComputationData
	{
		CloudData cloudData_source_dynamic;
		pcl::Correspondences pcl_correspondences;
		pcl::Correspondences pcl_correspondences_temp;
	};

	enum CorrespondenceComputationMethod
	{
		POINT_TO_POINT, POINT_TO_PLANE, POINT_TO_MLSSURFACE, DIRECT_POINT_PAIR
	};

	struct CorrespondencesComputationParameters
	{
		CorrespondenceComputationMethod method;
		float distanceThreshold;
		float normalAngleThreshold;
		bool boundaryTest;
		bool biDirectional;
		bool use_scpu;
		bool use_mcpu;
	};

	struct PairwiseRegistrationComputationData
	{
		Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_src;
		Eigen::Matrix<float, 3, Eigen::Dynamic> cloud_tgt;
	};

	enum PairwiseRegistrationComputationMethod
	{
		SVD, UMEYAMA
	};

	struct PairwiseRegistrationComputationParameters
	{
		PairwiseRegistrationComputationMethod method;
		bool allowScaling;
	};

	class PairwiseRegistration : public QObject
	{
		Q_OBJECT

	public:

		PairwiseRegistration(RegistrationData *target, RegistrationData *source, 
			QString registrationName, QObject *parent = 0);
		virtual ~PairwiseRegistration();

		virtual void initialize();
		virtual void initializeTransformation(const Eigen::Matrix4f &transformation);
		virtual void process(QVariantMap parameters);

		void estimateRMSErrorByTransformation(const Eigen::Matrix4f &transformation, float &rmsError, int &ovlNumber); 
		void estimateVirtualRMSErrorByTransformation(const Eigen::Matrix4f &transformation, float &rmsError, int &ovlNumber);

		inline RegistrationData* getTarget() {return target;}
		inline RegistrationData* getSource() {return source;}

		inline Eigen::Matrix4f getTransformation() {return transformation;}

		inline void setFreezed(bool freezed) {this->freezed = freezed;}
		inline bool getFreezed() {return freezed;}

		inline bool getErrorPrecomputed() {return errorPrecomputed;}

		inline float getRMSError() {return rmsError_total;}
		inline std::vector<float> getSquareErrors() {return squareErrors_total;}

		static inline QString generateName(QString targetName, QString sourceName) {return targetName + "<-" + sourceName;}

		static void preCorrespondences(RegistrationData *target, RegistrationData *source,
			const Eigen::Matrix4f &initialTransformation, CorrespondencesComputationParameters &correspondencesComputationParameters, 
			Correspondences &correspondences, CorrespondenceIndices &correspondenceIndices, 
			int &inverseStartIndex, CorrespondencesComputationData &correspondencesComputationData);

		static Eigen::Matrix4f registAr(Correspondences &correspondences, 
			PairwiseRegistrationComputationParameters pairwiseRegistrationComputationParameters, 
			PairwiseRegistrationComputationData &pairwiseRegistrationComputationData);

		static Eigen::Matrix4f icp(RegistrationData *target, RegistrationData *source, 
			const Eigen::Matrix4f &initialTransformation, CorrespondencesComputationParameters &correspondencesComputationParameters, 
			PairwiseRegistrationComputationParameters pairwiseRegistrationComputationParameters, int iterationNumber);

		static void computeSquareErrors(Correspondences &correspondences, std::vector<float> &squareErrors_total, float &rmsError_total);

	protected:
		Eigen::Matrix4f transformation;

		RegistrationData *target;
		RegistrationData *source;

		bool freezed;

		bool errorPrecomputed;
		float rmsError_total;
		std::vector<float> squareErrors_total;
	};

	class PairwiseRegistrationManager : public QObject
	{
		Q_OBJECT

	public:
		PairwiseRegistrationManager(QObject *parent = 0);
		virtual ~PairwiseRegistrationManager();
		PairwiseRegistration *addPairwiseRegistration(RegistrationData *target, RegistrationData *source, QString registrationName);
		void addPairwiseRegistration(PairwiseRegistration *pairwiseRegistration);
		void removePairwiseRegistration(QString registrationName);
		PairwiseRegistration *getPairwiseRegistration(QString registrationName);

		QList<PairwiseRegistration*> getAllPairwiseRegistrations();
		QStringList getAllPairwiseRegistrationNames();	
	};
}

#endif 