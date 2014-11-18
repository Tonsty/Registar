#ifndef VOXELGRID_H
#define VOXELGRID_H

#include <QtCore/QVariantMap>

#include "pclbase.h"

namespace registar
{
	class VoxelGrid
	{
	public:
		VoxelGrid();
		virtual ~VoxelGrid();

		static void filter(CloudDataPtr &cloudData, QVariantMap parameters, CloudDataPtr &cloudData_filtered);
	};
}

#endif