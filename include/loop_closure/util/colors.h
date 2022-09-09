#pragma once

#include <std_msgs/ColorRGBA.h>

class Colors {

    private: 
        static inline std_msgs::ColorRGBA get_color(int r, int g, int b, float a = 1) {
            std_msgs::ColorRGBA color;

            color.r = r / 255.0f;
            color.g = g / 255.0f;
            color.b = b / 255.0f;
            color.a = a;

            return color;
        }

    public:
        enum ColorNames {
            black,
            silver,
            gray,
            white,
            maroon,
            red,
            purple,
            fuchsia,
            green,
            lime,
            olive,
            yellow,
            navy,
            blue,
            teal,
            aqua
        };

        static std_msgs::ColorRGBA color_from_name(Colors::ColorNames color, float opacity = 1.0) {
            
            switch (color)
            {
                case ColorNames::aqua:
                    return get_color(0, 255, 255, opacity);
                    break;
                case ColorNames::black:
                    return get_color(0, 0, 0, opacity);
                    break;
                case ColorNames::blue:
                    return get_color(0, 0, 255, opacity);
                    break;
                case ColorNames::fuchsia:
                    return get_color(255, 0, 255, opacity);
                    break;
                case ColorNames::gray:
                    return get_color(128, 128, 128, opacity);
                    break;
                case ColorNames::green:
                    return get_color(0, 128, 0, opacity);
                    break;
                case ColorNames::lime:
                    return get_color(0, 255, 0, opacity);
                    break;
                case ColorNames::maroon:
                    return get_color(128, 0, 0, opacity);
                    break;
                case ColorNames::navy:
                    return get_color(0, 0, 128, opacity);
                    break;
                case ColorNames::olive:
                    return get_color(128, 128, 0, opacity);
                    break;
                case ColorNames::purple:
                    return get_color(128, 0, 128, opacity);
                    break;
                case ColorNames::red:
                    return get_color(255, 0, 0, opacity);
                    break;
                case ColorNames::teal:
                    return get_color(0, 128, 128, opacity);
                    break;
                case ColorNames::white:
                    return get_color(255, 255, 255, opacity);
                    break;
                case ColorNames::yellow:
                    return get_color(255, 255, 0, opacity);
                    break;
                default:
                    return std_msgs::ColorRGBA();
            }

        }

        Colors() = delete;
};