#pragma once

#include <Eigen/Eigen>
#include <queue>
#include <numeric>
#include "spectrum.hpp"

namespace ZR
{
	struct Photon {
		Eigen::Vector3d Pos; //位置
		Eigen::Vector3d Dir; //入射方向
		Spectrum power; //能量，通常用颜色值表示
		int axis;
	};

	struct Nearestphotons {
		Eigen::Vector3d Pos;
		int max_photons, found;
		bool got_heap;
		double* dist2;
		Photon** photons;
		Nearestphotons() {
			max_photons = found = 0;
			got_heap = false;
			dist2 = NULL;
			photons = NULL;
		}
		~Nearestphotons() {
			delete[] dist2;
			delete[] photons;
		}
	};

	class PhotonMap {
	public:
		PhotonMap();
		PhotonMap(int max);
		~PhotonMap();
		int maxPhotonNum;
		int PhotonNum;//光子数量
		Photon *mPhoton;
		void store(Photon pn);
		void MedianSplit(Photon* porg, int start, int end, int med, int axis);
		void balance();
		void balanceSegment(Photon*, int, int, int);
		bool getPhoton(Photon &pn, int index) {
			if (index > maxPhotonNum) return false;
			else {
				pn = mPhoton[index];
				return true;
			}
		}
		void getNearestPhotons(Nearestphotons* np, int index);
		double getPhotonPosAxis(int index, int axis) {
			return mPhoton[index].Pos[axis];
		}
		Spectrum getIrradiance(Eigen::Vector3d Pos, Eigen::Vector3d Norm, double max_dist, const int N);
		Eigen::Vector3d box_min, box_max;

	};
}