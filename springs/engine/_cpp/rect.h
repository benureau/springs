#ifndef RECT_H
#define RECT_H

#include "node.h"


using namespace std;

namespace springs {

  class Rect {
    public:
      double xL, xR, yB, yT, width, height, restitution;
      Rect(double xL, double xR, double yB, double yT, double restitution);
      bool collides(Node* node);
  };

  class Collision {
    public:
      Rect* rect;
      Node* node;

      double threshold, diff_v_x, diff_v_y;

      Collision(Rect* rect, Node* node, double threshold);
      void substep();

    protected:
      bool _x_not_y_collision, _disabled;
      double _bias;
  };

  class CollisionDetector {
    public:
      vector<Rect*> rects;
      int n_bins, n_bins_x, n_bins_y;
      double size_x, size_y;

      CollisionDetector(double size_x, double size_y);
      void add_rect(Rect* rect);
      void detect_collisions(vector<Node*> &nodes, double restitution_threshold,
                             vector<Collision> &collisions);

    protected:
      vector<vector<vector<Rect*>>> _bins;
      double _min_x_bin, _min_y_bin;
      bool _autosize_x, _autosize_y;

      void _prepare();
      // automatically chooses size_x and size_y based on the rectangles dimensions.
      void _autosize();
      int _bin_x(double x);
      int _bin_y(double x);
    };
}

#endif
