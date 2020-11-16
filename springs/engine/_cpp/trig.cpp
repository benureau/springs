#include <numeric>
#include <cmath>
#include <sys/types.h>
#include <iostream>

#include "trig.h"


namespace springs {

    /* Segment */

    Segment::Segment(const double x1, const double y1, const double x2, const double y2)
        : x1(x1), y1(y1), x2(x2), y2(y2) {
        double v12_x = x2 - x1;
        double v12_y = y2 - y1;
        length = sqrt(v12_x * v12_x + v12_y * v12_y);

        tangent_x = v12_x / length;
        tangent_y = v12_y / length;

        normal_x = - tangent_y;
        normal_y =   tangent_x;
    }

    /* Change the direction of the normal and tangent */
    void Segment::rotate() {
        normal_x  = - normal_x;
        normal_y  = - normal_y;
        tangent_x = - tangent_x;
        tangent_y = - tangent_y;
    }

    double Segment::dot_normal(Node* node) {
        return (node->x - x1) * normal_x + (node->y - y1) * normal_y;
    }

    /* Triangle */

    Triangle::Triangle(double xA, double yA, double xB, double yB, double xC, double yC,
                       double restitution)
        : xA(xA), yA(yA), xB(xB), yB(yB), xC(xC), yC(yC), restitution(restitution),
          segment_AB(xA, yA, xB, yB), segment_BC(xB, yB, xC, yC), segment_CA(xC, yC, xA, yA) {
        x_min = fmin(xA, fmin(xB, xC));
        x_max = fmax(xA, fmax(xB, xC));
        y_min = fmin(yA, fmin(yB, yC));
        y_max = fmax(yA, fmax(yB, yC));

        // setting segments' normal correctly outward.
        if ((xC - xB) * segment_AB.normal_x + (yC - yB) * segment_AB.normal_y >= 0) {
            segment_AB.rotate();
        }
        if ((xA - xC) * segment_BC.normal_x + (yA - yC) * segment_BC.normal_y >= 0) {
            segment_BC.rotate();
        }
        if ((xB - xA) * segment_CA.normal_x + (yB - yA) * segment_CA.normal_y >= 0) {
            segment_CA.rotate();
        }
    }

    void Triangle::collides(Node* node, Contact &contact) {
        contact.active = false;

        if (!(x_min < node->x && node->x <= x_max &&
              y_min < node->y && node->y <= y_max   )) {
            return;
        }

        const double dot_AB = segment_AB.dot_normal(node);
        // std::cout << dot_AB << std::endl;
        if (dot_AB > 0) { return; }
        // else if (dot_AB == 0) {  // assumed on the segment, because in the AABB too.
        //     contact.active  = true;
        //     contact.segment = &segment_AB;
        //     contact.node = node;
        //     contact.restitution = restitution;
        //     return;
        // }
        const double dot_BC = segment_BC.dot_normal(node);
        // std::cout << dot_BC << std::endl;
        if (dot_BC > 0) { return; }
        else if (dot_BC == 0) {  // assumed on the segment, because in the AABB too.
            contact.active  = true;
            contact.segment = &segment_BC;
            contact.node = node;
            contact.restitution = restitution;
            return;
        }
        const double dot_CA = segment_CA.dot_normal(node);
        // std::cout << dot_CA << std::endl;
        if (dot_CA > 0) { return; }
        else if (dot_CA == 0) {  // assumed on the segment, because in the AABB too.
            contact.active  = true;
            contact.segment = &segment_CA;
            contact.node = node;
            contact.restitution = restitution;
            return;
        }

        // If here, the point is inside the triangle
        contact.active  = true;
        contact.node = node;
        contact.restitution = restitution;
        if (dot_BC > dot_CA) {
            node->x -= dot_BC * segment_BC.normal_x;
            node->y -= dot_BC * segment_BC.normal_y;
            contact.segment = &segment_BC;
        } else {
            node->x -= dot_CA * segment_CA.normal_x;
            node->y -= dot_CA * segment_CA.normal_y;
            contact.segment = &segment_CA;
        }

        // if (dot_AB > dot_BC) {
        //     if (dot_AB > dot_CA) {
        //         node->x -= dot_AB * segment_AB.normal_x;
        //         node->y -= dot_AB * segment_AB.normal_y;
        //         contact.segment = &segment_AB;
        //     } else {
        //         node->x -= dot_CA * segment_CA.normal_x;
        //         node->y -= dot_CA * segment_CA.normal_y;
        //         contact.segment = &segment_CA;
        //     }
        // } else {
        //     if (dot_BC > dot_CA) {
        //         node->x -= dot_BC * segment_BC.normal_x;
        //         node->y -= dot_BC * segment_BC.normal_y;
        //         contact.segment = &segment_BC;
        //     } else {
        //         node->x -= dot_CA * segment_CA.normal_x;
        //         node->y -= dot_CA * segment_CA.normal_y;
        //         contact.segment = &segment_CA;
        //     }
        // }
    }


    /* Contact */

    Contact::Contact(double threshold)
        : threshold(threshold), diff_vn(0), diff_vt(0), active(false)
    {}

    void Contact::reset() {
      diff_vn = 0;
      diff_vt = 0;
      active = false;
    }

    inline void Contact::prepare() {
      // should only be done once per step
      x = node->x;
      y = node->y;
      const double vn = node->v_x * segment->normal_x + node->v_y * segment->normal_y;
      bias = vn * restitution;
    }

    inline void Contact::substep() {
        if (!active) { return; }

        // tangent: friction
        const double vt = node->v_x * segment->tangent_x + node->v_y * segment->tangent_y;
        double max_friction = node->friction * fabs(diff_vn);
        // std::cout << "friction " << max_friction << std::endl;
        if (fabs(vt) > 1) { max_friction /= 2; }  // moving: dynamic friction
        double new_diff_vt = clamp(-max_friction, diff_vt - vt, max_friction);
        node->v_x += (new_diff_vt - diff_vt) * segment->tangent_x;
        node->v_y += (new_diff_vt - diff_vt) * segment->tangent_y;
        diff_vt = new_diff_vt;
        // normal: restitution
        const double vn = node->v_x * segment->normal_x + node->v_y * segment->normal_y;
        const double new_diff_vn = diff_vn - vn - bias;
        node->v_x += (new_diff_vn - diff_vn) * segment->normal_x;
        node->v_y += (new_diff_vn - diff_vn) * segment->normal_y;
        diff_vn = new_diff_vn;
    }


    TriangleCollisionDetector::TriangleCollisionDetector(double size_x, double size_y)
        : size_x(size_x), size_y(size_y)
    {
        _autosize_x = size_x <= 0;
        _autosize_y = size_y <= 0;
    }

    inline void TriangleCollisionDetector::add_triangle(Triangle* triangle) {
        triangles.push_back(triangle);
        _bins.clear();
    }

    inline int TriangleCollisionDetector::_bin_x(double x) {
        return floor((x - _min_x_bin) / size_x);
    }
    inline int TriangleCollisionDetector::_bin_y(double y) {
        return floor((y - _min_y_bin) / size_y);
    }

    inline void TriangleCollisionDetector::_autosize() {
        if (_autosize_x) {
            vector<double> widths;
            for (auto& triangle: triangles) { widths.push_back(triangle->x_max - triangle->x_min); }
            double mean_width = accumulate(widths.begin(), widths.end(), 0.0) / widths.size();
            size_x = 3 * mean_width; // heuristic. TODO: refine.
        }
        if (_autosize_y) {
            vector<double> heights;
            for (auto& triangle: triangles) { heights.push_back(triangle->y_max - triangle->y_min); }
            double mean_height = accumulate(heights.begin(), heights.end(), 0.0) / heights.size();
            size_y = 3 * mean_height; // heuristic. TODO: refine.
        }
    }

    inline void TriangleCollisionDetector::_prepare() {
        _autosize();

        vector<double> x_maxs, x_mins, y_maxs, y_mins;
        for (auto& triangle: triangles) {
            x_mins.push_back(triangle->x_min); x_maxs.push_back(triangle->x_max);
            y_mins.push_back(triangle->y_min); y_maxs.push_back(triangle->y_max);
        }
        double min_x = *min_element(x_mins.begin(), x_mins.end());
        double max_x = *max_element(x_maxs.begin(), x_maxs.end());
        double min_y = *min_element(y_mins.begin(), y_mins.end());
        double max_y = *max_element(y_maxs.begin(), y_maxs.end());

        _min_x_bin = size_x * floor(min_x / size_x);
        _min_y_bin = size_y * floor(min_y / size_y);
        n_bins_x = floor(max_x / size_x) - floor(min_x / size_x) + 1;
        n_bins_y = floor(max_y / size_y) - floor(min_y / size_y) + 1;
        n_bins = n_bins_x * n_bins_y;

        for (int i = 0; i < n_bins_x; i++) {
            _bins.push_back(vector<vector<Triangle*>>());
            for (int j = 0; j < n_bins_y; j++) {
                _bins[i].push_back(vector<Triangle*>());
            }
        }

        for (auto& triangle: triangles) {
            for (int i = _bin_x(triangle->x_min); i <= _bin_x(triangle->x_max); i++) {
                for (int j = _bin_y(triangle->y_min); j <= _bin_y(triangle->y_max); j++) {
                    _bins[i][j].push_back(triangle);
                }
            }
        }
    }

    inline void TriangleCollisionDetector::detect_collisions(vector<Node*> &nodes,
                                                             double restitution_threshold,
                                                             vector<Contact> &contacts) {
        if (triangles.size() == 0 || nodes.size() == 0) { return; }
        if (_bins.size() == 0) { _prepare(); }

        Contact contact = Contact(restitution_threshold);
        for (Node* node: nodes) {
            int bin_x = _bin_x(node->x);
            int bin_y = _bin_y(node->y);
            if (0 <= bin_y && bin_y < n_bins_y && 0 <= bin_x && bin_x < n_bins_x) {
                for (Triangle* triangle: triangles) {
                    triangle->collides(node, contact);
                    if (contact.active) { node->colliding = true;
                                          contact.prepare(); contacts.push_back(contact);
                                          contact.reset(); }
                }
            }
        }
    }
}
