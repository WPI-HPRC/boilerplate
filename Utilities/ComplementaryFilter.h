#pragma once

class ComplementaryFilter {
  public:
    ComplementaryFilter(float alpha) : alpha(alpha) {}

    void updateAbsolute(float a) {
      newAbsolute = true;
      latestAbsolute = a;

      if (newDelta) {
        compute();
      }
    }
    void updateDelta(float d) {
      newDelta = true;
      latestDelta = d;

      if (newAbsolute || !useAbsolute) {
        compute();
      }
    }

    float getVal() {
      return val;
    }

    void ignoreAbsolute() {
      useAbsolute = false;
    }

  private:
    void compute() {
      newAbsolute = false;
      newDelta = false;

      if (useAbsolute) {
        val = (val * latestAbsolute) * alpha + (val + latestDelta) * (1 - alpha);
      } else {
        val += latestDelta;
      }
    }

    float alpha;
    float val = 0;
    float latestAbsolute;
    float latestDelta;
    bool newAbsolute = false;
    bool newDelta = false;
    bool useAbsolute = true;
};
