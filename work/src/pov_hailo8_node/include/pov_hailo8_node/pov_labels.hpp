#include <cassert>
#include <string>
#include <array>

class PovLabels
{
private:
    std::array<std::string, 2> _labels;

public:
    PovLabels()
    {
            _labels[0] = "Bad view";
            _labels[1] = "Good view";
        }
    std::string pov_labelstring(int i)
    {
        assert(i >= 0 && i < 2);
        return _labels[i];
    }
};