#ifndef HYPERSPECTRALSLICEDISPLAYADAPTER_H
#define HYPERSPECTRALSLICEDISPLAYADAPTER_H

#include <qImageDisplayWidget/imageadapter.h>

#include <MultidimArrays/MultidimArrays.h>

#include <LibStevi/utils/types_manipulations.h>

template<typename Array_T, Multidim::ArrayDataAccessConstness viewConstness = Multidim::NonConstView>
class HyperspectralSliceDisplayAdapter : public QImageDisplay::ImageAdapter
{
public:

    HyperspectralSliceDisplayAdapter(Multidim::Array<Array_T, 3, viewConstness> const* array,
                        Array_T blackLevel = StereoVision::TypesManipulations::defaultBlackLevel<Array_T>(),
                        Array_T whiteLevel = StereoVision::TypesManipulations::defaultWhiteLevel<Array_T>(),
                        int xAxis = 1,
                        int yAxis = 0,
                        int channelAxis = 2,
                        int sliceChannel = 0,
                        QObject* parent = nullptr) :
        QImageDisplay::ImageAdapter(parent),
        _array(array),
        _x_axis(xAxis),
        _y_axis(yAxis),
        _channel_axis(channelAxis),
        _slice_channel(sliceChannel),
        _black_level(blackLevel),
        _white_level(whiteLevel),
        _displayOriginalChannels(false)
    {

    }

    QSize getImageSize() const override {
        if (_array == nullptr) {
            return QSize();
        }
        return QSize(_array->shape()[_x_axis], _array->shape()[_y_axis]);
    }

    QColor getColorAtPoint(int x, int y) const override{

        if (_array == nullptr) {
            return QColor();
        }

        std::array<int, 3> idx;
        idx[_x_axis] = x;
        idx[_y_axis] = y;

        QColor ret;

        idx[_channel_axis] = _slice_channel;
        Array_T value = _array->valueOrAlt(idx, 0);
        ret.setRed(valueToColor(value));
        ret.setGreen(valueToColor(value));
        ret.setBlue(valueToColor(value));

        return ret;
    }

    QVector<ChannelInfo> getOriginalChannelsInfos(QPoint const& pos) const override {

        if (!_displayOriginalChannels) {
            return QImageDisplay::ImageAdapter::getOriginalChannelsInfos(pos);
        }

        QVector<ChannelInfo> ret(1);

        QString formatStr = "%1 ";

        std::array<int, 3> idx;
        idx[_x_axis] = pos.x();
        idx[_y_axis] = pos.y();
        idx[_channel_axis] = _slice_channel;

        ret[0].channelName = _channelName;

        double tmp = _array->valueOrAlt(idx, 0);

        if (std::is_floating_point_v<Array_T>) {
            ret[0].channelValue = QString(formatStr).arg(tmp, 0, 'g', 3);
        } else {
            ret[0].channelValue = QString(formatStr).arg(_array->valueOrAlt(idx, 0));
        }

        return ret;

    }

    inline void configureOriginalChannelDisplay( QString const& channelName) {
        _displayOriginalChannels = true;
        _channelName = channelName;
    }

    inline void clearOriginalChannelDisplay() {
        _displayOriginalChannels = false;
        _channelName.clear();
    }

    void nextChannel() {
        _slice_channel++;
        _slice_channel %= _array->shape()[_channel_axis];

        Q_EMIT imageValuesChanged(QRect());
    }

    void previousChannel() {
        _slice_channel--;
        if (_slice_channel < 0) {
            _slice_channel += _array->shape()[_channel_axis];
        }
        _slice_channel %= _array->shape()[_channel_axis];

        Q_EMIT imageValuesChanged(QRect());
    }

    int getChannel() {
        return _slice_channel;
    }

protected:

    using ComputeType = StereoVision::TypesManipulations::accumulation_extended_t<Array_T>;

    inline uint8_t valueToColor(Array_T const& value) const {

        if (value < _black_level) {
            return 0;
        }

        if (value >= _white_level) {
            return 255;
        }

        ComputeType transformed = (255*(static_cast<ComputeType>(value) - static_cast<ComputeType>(_black_level)))
                /(_white_level - _black_level);

        return static_cast<uint8_t>(transformed);
    }

    Multidim::Array<Array_T, 3, viewConstness> const* _array;

    int _slice_channel;

    int _x_axis;
    int _y_axis;
    int _channel_axis;

    Array_T _black_level;
    Array_T _white_level;

    bool _displayOriginalChannels;
    QString _channelName;

};

#endif // HYPERSPECTRALSLICEDISPLAYADAPTER_H
