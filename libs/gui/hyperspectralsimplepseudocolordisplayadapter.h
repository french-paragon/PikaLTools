#ifndef HYPERSPACTRALSIMPLEPSEUDOCOLORDISPLAYADAPTER_H
#define HYPERSPACTRALSIMPLEPSEUDOCOLORDISPLAYADAPTER_H

#include <StereoVision/QImageDisplayWidget/imageadapter.h>

#include <MultidimArrays/MultidimArrays.h>

#include <StereoVision/utils/types_manipulations.h>

template<typename Array_T, Multidim::ArrayDataAccessConstness viewConstness = Multidim::NonConstView>
class HyperspectralSimplePseudocolorDisplayAdapter : public QImageDisplay::ImageAdapter
{
public:

    HyperspectralSimplePseudocolorDisplayAdapter(Multidim::Array<Array_T, 3, viewConstness> const* array,
                                                 Array_T blackLevel = StereoVision::TypesManipulations::defaultBlackLevel<Array_T>(),
                                                 Array_T whiteLevel = StereoVision::TypesManipulations::defaultWhiteLevel<Array_T>(),
                                                 int xAxis = 1,
                                                 int yAxis = 0,
                                                 int channelAxis = 2,
                                                 std::array<int,3> sliceChannels = {},
                        QObject* parent = nullptr) :
        QImageDisplay::ImageAdapter(parent),
        _array(array),
        _x_axis(xAxis),
        _y_axis(yAxis),
        _channel_axis(channelAxis),
        _slice_channel_rgb(sliceChannels),
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

        idx[_channel_axis] = _slice_channel_rgb[0];
        Array_T value = _array->valueOrAlt(idx, 0);
        ret.setRed(valueToColor(value));

        idx[_channel_axis] = _slice_channel_rgb[1];
        value = _array->valueOrAlt(idx, 0);
        ret.setGreen(valueToColor(value));

        idx[_channel_axis] = _slice_channel_rgb[2];
        value = _array->valueOrAlt(idx, 0);
        ret.setBlue(valueToColor(value));

        return ret;
    }

    QVector<ChannelInfo> getOriginalChannelsInfos(QPoint const& pos) const override {

        if (!_displayOriginalChannels) {
            return QImageDisplay::ImageAdapter::getOriginalChannelsInfos(pos);
        }

        QVector<ChannelInfo> ret(3);

        QString formatStr = "%1 ";

        std::array<int, 3> idx;
        idx[_x_axis] = pos.x();
        idx[_y_axis] = pos.y();

        for (int c = 0; c < 3; c++) {
            idx[_channel_axis] = _slice_channel_rgb[c];

            ret[c].channelName = _channelName[c];

            double tmp = _array->valueOrAlt(idx, 0);

            if (std::is_floating_point_v<Array_T>) {
                ret[c].channelValue = QString(formatStr).arg(tmp, 0, 'g', 3);
            } else {
                ret[c].channelValue = QString(formatStr).arg(_array->valueOrAlt(idx, 0));
            }
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

    std::array<int, 3> getChannels() const {
        return _slice_channel_rgb;
    }

    void setChannels(std::array<int, 3> const& channels) {
        if (channels != _slice_channel_rgb) {
            _slice_channel_rgb = channels;
            Q_EMIT imageValuesChanged(QRect());
        }
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

    std::array<int, 3> _slice_channel_rgb;

    int _x_axis;
    int _y_axis;
    int _channel_axis;

    Array_T _black_level;
    Array_T _white_level;

    bool _displayOriginalChannels;
    QString _channelName;

};

#endif // HYPERSPACTRALSIMPLEPSEUDOCOLORDISPLAYADAPTER_H
