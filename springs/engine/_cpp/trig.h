#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "node.h"


using namespace std;

namespace springs {

  // //
  // class Collision {
  //   Node* node;
  //   Contact* contact;
  // };

  class Segment {
    public:
      double x1, y1, x2, y2, length;
      double normal_x,  normal_y;
      double tangent_x, tangent_y;

      Segment(const double x1, const double y1, const double x2, const double y2);
      void rotate();
      double dot_normal(Node* node);
  };

  class Contact {
    public:
      double x, y;
      double restitution, threshold, diff_vn, diff_vt, bias;

      Node* node;
      Segment* segment;
      bool active;

      Contact(double threshold);
      void prepare();
      void reset();
      void substep();
  };

  class Triangle {
    public:
      double xA, yA, xB, yB, xC, yC, restitution;
      Segment segment_AB, segment_BC, segment_CA;

      Triangle(double xA, double yA, double xB, double yB, double xC, double yC, double restitution);
      void collides(Node* node, Contact &contact);

      double x_min, x_max, y_min, y_max;
  };

  class TriangleCollisionDetector {
    public:
      // vector<Nodes*> nodes;
      vector<Triangle*> triangles;
      int n_bins, n_bins_x, n_bins_y;
      double size_x, size_y;

      TriangleCollisionDetector(double size_x, double size_y);
      void add_node(Node* node);
      void add_triangle(Triangle* trig);
      void detect_collisions(vector<Node*> &nodes, double restitution_threshold,
                             vector<Contact> &contacts);

    protected:
      vector<vector<vector<Triangle*>>> _bins;
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
