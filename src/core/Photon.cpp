#include "Photon.hpp"
#include "ZRender.hpp"


namespace ZR
{
	PhotonMap::PhotonMap() {
		maxPhotonNum = 10000;
		PhotonNum = 0;
		mPhoton = new Photon[10000 + 1];
		box_min = Eigen::Vector3d(ZR::Infinity, ZR::Infinity, ZR::Infinity);
		box_max = Eigen::Vector3d(-ZR::Infinity, -ZR::Infinity, -ZR::Infinity);
	}
	PhotonMap::PhotonMap(int max) {
		maxPhotonNum = max;
		PhotonNum = 0;
		mPhoton = new Photon[max + 1];
		box_min = Eigen::Vector3d(ZR::Infinity, ZR::Infinity, ZR::Infinity);
		box_max = Eigen::Vector3d(-ZR::Infinity, -ZR::Infinity, -ZR::Infinity);
	}
	PhotonMap::~PhotonMap() {

	}
	void PhotonMap::store(Photon photon) {
		if (PhotonNum >= maxPhotonNum) return;

		photon.Dir.normalize();
		mPhoton[++PhotonNum] = photon;
		box_min = Eigen::Vector3d(std::min(box_min.x(), photon.Pos.x()), std::min(box_min.y(), photon.Pos.y()), std::min(box_min.z(), photon.Pos.z()));
		box_max = Eigen::Vector3d(std::max(box_max.x(), photon.Pos.x()), std::max(box_max.y(), photon.Pos.y()), std::max(box_max.z(), photon.Pos.z()));
	}

	int calMed(int start, int end) {
		int num = end - start + 1;
		int med;
		int as = 1, b = 2;
		while (as < num) {
			as += b;
			b *= 2;
		}
		if (as == num)
			return start + num / 2;
		b /= 2;
		if (as - b / 2 < num) {
			return start + as / 2;
		}
		else
			return start + as / 2 - (as - b / 2 - num);
	}

	void PhotonMap::MedianSplit(Photon* tempPhoton, int start, int end, int med, int axis) {
		int l = start, r = end;
		while (l < r) {
			double key = tempPhoton[r].Pos[axis];
			int i = l - 1, j = r;
			for (; ; ) {
				while (tempPhoton[++i].Pos[axis] < key);
				while (tempPhoton[--j].Pos[axis] > key && j > l);
				if (i >= j) break;
				std::swap(tempPhoton[i], tempPhoton[j]);
			}
			std::swap(tempPhoton[i], tempPhoton[r]);
			if (i >= med) r = i - 1;
			if (i <= med) l = i + 1;
		}
	}

	void PhotonMap::balance() {
		Photon* tempPhoton = new Photon[PhotonNum + 1];

		for (int i = 1; i <= PhotonNum; i++)
			tempPhoton[i] = mPhoton[i];

		balanceSegment(tempPhoton, 1, 1, PhotonNum);
		delete[] tempPhoton;
	}

	void PhotonMap::balanceSegment(Photon* tempPhoton, int index, int start, int end) {
		if (start == end) {
			mPhoton[index] = tempPhoton[start];
			mPhoton[index].axis = 100;
			return;
		}

		int med = calMed(start, end);

		int axis = 2;
		if (box_max.x() - box_min.x() > box_max.y() - box_min.y() && box_max.x() - box_min.x() > box_max.z() - box_min.z())
			axis = 0;
		else
		if (box_max.y() - box_min.y() > box_max.z() - box_min.z()) axis = 1;

		MedianSplit(tempPhoton, start, end, med, axis);
		mPhoton[index] = tempPhoton[med];
		mPhoton[index].axis = axis;

		if (start < med) {
			double tmp = box_max[axis];
			box_max[axis] = mPhoton[index].Pos[axis];
			balanceSegment(tempPhoton, index * 2, start, med - 1);
			box_max[axis] = tmp;
		}

		if (med < end) {
			double tmp = box_min[axis];
			box_min[axis] = mPhoton[index].Pos[axis];
			balanceSegment(tempPhoton, index * 2 + 1, med + 1, end);
			box_min[axis] = tmp;
		}
	}

	void PhotonMap::getNearestPhotons(Nearestphotons* np, int index) {
		if (index > PhotonNum) return;
		Photon *photon = &mPhoton[index];

		if (index * 2 <= PhotonNum) {
			double dist = np->Pos[photon->axis] - photon->Pos[photon->axis];
			if (dist < 0) {
				getNearestPhotons(np, index * 2);
				if (dist * dist < np->dist2[0]) getNearestPhotons(np, index * 2 + 1);
			}
			else {
				getNearestPhotons(np, index * 2 + 1);
				if (dist * dist < np->dist2[0]) getNearestPhotons(np, index * 2);
			}
		}

		double a1,a2,a3;
		a1 = (photon->Pos-np->Pos).x();
		a2 = (photon->Pos-np->Pos).y();
		a3 = (photon->Pos-np->Pos).z();

		double dist2 = (a1*a1)+(a2*a2)+(a3*a3);
		if (dist2 > np->dist2[0]) return;

		if (np->found < np->max_photons) {
			np->found++;
			np->dist2[np->found] = dist2;
			np->photons[np->found] = photon;
		}
		else {
			if (np->got_heap == false) {
				for (int i = np->found >> 1; i >= 1; i--) {
					int par = i;
					Photon* tmp_photon = np->photons[i];
					double tmp_dist2 = np->dist2[i];
					while ((par << 1) <= np->found) {
						int j = par << 1;
						if (j + 1 <= np->found && np->dist2[j] < np->dist2[j + 1]) j++;
						if (tmp_dist2 >= np->dist2[j]) break;

						np->photons[par] = np->photons[j];
						np->dist2[par] = np->dist2[j];
						par = j;
					}
					np->photons[par] = tmp_photon;
					np->dist2[par] = tmp_dist2;
				}
				np->got_heap = true;
			}

			int par = 1;
			while ((par << 1) <= np->found) {
				int j = par << 1;
				if (j + 1 <= np->found && np->dist2[j] < np->dist2[j + 1]) j++;
				if (dist2 > np->dist2[j]) break;

				np->photons[par] = np->photons[j];
				np->dist2[par] = np->dist2[j];
				par = j;
			}
			np->photons[par] = photon;
			np->dist2[par] = dist2;

			np->dist2[0] = np->dist2[1];
		}
	}

	Spectrum PhotonMap::getIrradiance(Eigen::Vector3d Pos, Eigen::Vector3d Norm, double max_dist, const int N) {
		Spectrum ret;
		Nearestphotons np;
		np.Pos = Pos;
		np.max_photons = N;
		np.dist2 = new double[N + 1];
		np.photons = new Photon*[N + 1];
		np.dist2[0] = max_dist * max_dist;
		getNearestPhotons(&np, 1);
		if (np.found <= 8) return ret;


		//这个最大距离是光子距离找到的N个光子的最大距离，而
		//max_dist是我们设置的最大距离
		double maxDist = sqrt(np.dist2[0]);
		const double k = 1.1;
		for (int i = 1; i <= np.found; i++) {
			Eigen::Vector3d dir = np.photons[i]->Dir;
			double wht = 1.0 - (np.Pos - np.photons[i]->Pos).norm() / (k * maxDist);

			ret += wht * np.photons[i]->power;
		}
		ret = ret * (1.0 / (np.dist2[0] * (1 - 2.0f / (3 * k))  ));
		//std::cerr<<ret[0]<<"   "<<ret[1]<<"  "<<ret[2]<<"\n";
		return ret;
	}
}