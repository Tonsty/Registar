#ifndef PAIRWISEREGISTRATIONINTERACTOR_H
#define PAIRWISEREGISTRATIONINTERACTOR_H

#include <QtCore/QVariantMap>

#include "pairwiseregistration.h"

class CloudVisualizer;

class PairwiseRegistrationInteractor : public PairwiseRegistration
{
	Q_OBJECT

public:

	PairwiseRegistrationInteractor(RegistrationData *target, RegistrationData *source,
		QString registrationName, QObject *parent = 0);
	virtual ~PairwiseRegistrationInteractor();

	virtual void initialize();
	virtual void initializeTransformation(const Eigen::Matrix4f &transformation);
	virtual void process(QVariantMap parameters);

	inline void setCloudVisualizer(CloudVisualizer *cloudVisualizer) {this->cloudVisualizer = cloudVisualizer;}
	inline CloudVisualizer* getCloudVisualizer() {return cloudVisualizer;}

protected:
	CloudVisualizer *cloudVisualizer;
private:
	void renderErrorMap(CorrespondenceIndices &correspondenceIndices, int &inverseStartIndex, std::vector<float> &squareErrors_total);
	void exportTransformation();	
};

#endif