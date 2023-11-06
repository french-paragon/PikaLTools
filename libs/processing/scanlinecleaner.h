#ifndef SCANLINECLEANER_H
#define SCANLINECLEANER_H

#include <vector>
#include <cmath>

#include <Eigen/Core>

/*!
 * \brief The scanlinecleaner class provide a configurable functor to isolate scanlines in a pushbrum trajectory.
 */
template<typename T>
class scanlinecleaner {

public:

    scanlinecleaner(T std_dev = 1,
                    int filtRadius = -1,
                    T speedTresh = 0.2,
                    T deltaTresh = 0.2,
                    int maskExpend = 1) :
        _filt_std_dev(std_dev),
        _speedTresh(speedTresh),
        _deltaTresh(deltaTresh),
        _maskExpand(maskExpend),
        _filter((filtRadius <= 0) ? 2*int(2*std::abs(std_dev))+1 : 2*filtRadius+1)
    {
        recomputeFilter();
    }

    inline int filterRadius() const {
        return (_filter.size()-1)/2;
    }

    inline void setFilterRadius(int radius) {
        int s = 2*radius+1;

        if (_filter.size() == s) {
            return;
        }

        _filter.resize(s);
        recomputeFilter();
    }

    inline T filterStandardDeviation() const {
        return _filt_std_dev;
    }

    inline void setFilterStadardDeviation(T stdDev) {
        if (_filt_std_dev != stdDev) {
            _filt_std_dev = stdDev;
            recomputeFilter();
        }
    }

    inline T speedThreshold() const {
        return _speedTresh;
    }

    inline void setSpeedThreshold(T threshold) {
        _speedTresh = threshold;
    }

    inline T deltaThreshold() const {
        return _deltaTresh;
    }

    inline void setDeltaThreshold(T threshold) {
        _deltaTresh = threshold;
    }

    inline int maskExpend() const {
        return _maskExpand;
    }

    inline void setMaskExpend(int expend) {
        _maskExpand = expend;
    }

    std::vector<bool> detectCleanLines(std::vector<Eigen::Vector<T,3>> const& trajectory) {

        int r = filterRadius();
        int s = trajectory.size();

        if (s <= 0) {
            return std::vector<bool>();
        }

        std::vector<Eigen::Vector<T,3>> diff(s);

        for(int i = 0; i < s; i++) {
            diff[i] = Eigen::Vector<T,3>::Zero();

            for (int j = 0; j < _filter.size(); j++) {

                int id = i+j-r;

                if (id < 0) {
                    id = 0;
                }

                if (id >= trajectory.size()) {
                    id = trajectory.size()-1;
                }

                diff[i] += trajectory[id]*_filter[j];
            }

        }

        std::vector<T> speed(s);
        std::vector<T> delta(s);

        for (int i = 0; i < s; i++) {
            speed[i] = diff[i].norm();

            Eigen::Vector<T,3> tmp = Eigen::Vector<T,3>::Zero();

            for (int d : {-1,1}) {
                T scale = (d < 0) ? -1 : 1;

                int id = i+d;

                if (id < 0) {
                    id = 0;
                }

                if (id >= trajectory.size()) {
                    id = trajectory.size()-1;
                }

                tmp += scale*trajectory[id].normalize();
            }

            delta[i] = tmp.norm();
        }


        std::vector<T> speed_copy = speed;
        std::vector<T> delta_copy = delta;

        int medianPos = s/2;

        std::nth_element(speed_copy.begin(), speed_copy.begin()+medianPos, speed_copy.end());
        std::nth_element(delta_copy.begin(), delta_copy.begin()+medianPos, delta_copy.end());

        T medianSpeed = speed_copy[medianPos];
        T medianDelta = delta_copy[medianPos];

        T maxSpeed = speed_copy[medianPos];
        T maxDelta = delta_copy[medianPos];

        T speedRange = (1-_speedTresh)*medianSpeed/maxSpeed;
        T deltaRange = (1-_speedTresh)*medianSpeed/maxSpeed;

        std::vector<bool> tmp(s);

        for (int i = 0; i < s; i++) {
            tmp[i] = std::abs(speed[i] - medianSpeed) < speedRange and delta[i] < medianDelta+deltaRange;
        }

        std::vector<bool> ret(s);

        for (int i = 0; i < s; i++) {

            bool mask = false;

            for (int d = -_maskExpand; d <= _maskExpand; d++) {

                int id = i+d;

                if (id < 0) {
                    id = 0;
                }

                if (id >= s) {
                    id = s-1;
                }

                if (tmp[id] == true) {
                    mask=true;
                }
            }
        }

        return ret;

    }

protected:

    void recomputeFilter() {

        int r = filterRadius();

        T sum = 0;
        T var = _filt_std_dev*_filt_std_dev;

        for (int i = 0; i < _filter.size(); i++) {

            int id = i-r;
            T val = std::exp(-(id*id)/var);

            _filter[i] = -2*id*val;

            sum += val;
        }

        for (int i = 0; i < _filter.size(); i++) {
            _filter[i] /= sum;
        }

    }

    T _filt_std_dev;
    T _filtThreshold;

    T _speedTresh;
    T _deltaTresh;

    int _maskExpand;

    std::vector<T> _filter;

};

#endif // SCANLINECLEANER_H
