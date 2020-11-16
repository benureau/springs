#include <numeric>
#include <cmath>
#include <sys/types.h>
#include <iostream>

#include "rect.h"


namespace springs {

    Rect::Rect(double xL, double xR, double yB, double yT, double restitution)
        : xL(xL), xR(xR), yB(yB), yT(yT), restitution(restitution) {
        width  = xR - xL;  // FIXME: necessary in C++?
        height = yT - yB;
    }

    inline bool Rect::collides(Node* node) {
        return (xL < node->x && node->x <= xR &&
                yB < node->y && node->y <= yT   );
    }

    Collision::Collision(Rect* rect, Node* node, double threshold)
        : rect(rect), node(node), threshold(threshold), diff_v_x(0), diff_v_y(0), _disabled(false)
    {


        // determine collision point and bias.
        double diff_xL = node->x - rect->xL;
        double diff_xR = rect->xR - node->x;
        double diff_yB = node->y - rect->yB;
        double diff_yT = rect->yT - node->y;

        _x_not_y_collision = fmin(diff_xL, diff_xR) < fmin(diff_yB, diff_yT);
        if (_x_not_y_collision) {
            if (diff_xL < diff_xR) {
                node->x = rect->xL;
                if (node->v_x < 0) {_disabled = true;}
                else if (node->v_x < threshold) { _bias = 0.0; }
                else { _bias = - node->v_x * rect->restitution; }
            } else {
                node->x = rect->xR;
                if (node->v_x > 0) {_disabled = true;}
                else if (node->v_x > -threshold) { _bias = 0.0; }
                else { _bias = - node->v_x * rect->restitution; }
            }
        } else {
            if (diff_yB < diff_yT) {
                node->y = rect->yB;
                if (node->v_y < 0) {_disabled = true;}
                else if (node->v_y < threshold) { _bias = 0.0; }
                else { _bias = - node->v_y * rect->restitution; }
            } else {
                node->y = rect->yT;
                if (node->v_y > 0) {_disabled = true;}
                else if (node->v_y > -threshold) { _bias = 0.0; }
                else { _bias = - node->v_y * rect->restitution; }
            }
        }
    }

    inline void Collision::substep() {
        if (_disabled) { return; }

        if (_x_not_y_collision) {
            // friction
            double max_friction = node->friction * fabs(diff_v_x);
            if (fabs(node->v_y) > 1) { max_friction /= 2; }  // moving: dynamic friction
            double new_diff_v_y = clamp(-max_friction, diff_v_y - node->v_y, max_friction);
            node->v_y += new_diff_v_y - diff_v_y;
            diff_v_y = new_diff_v_y;
            // restitution
            double new_diff_v_x = diff_v_x + _bias - node->v_x;
            node->v_x += new_diff_v_x - diff_v_x;
            diff_v_x = new_diff_v_x;

        } else {
            // friction
            double max_friction = node->friction * fabs(diff_v_y);
            if (fabs(node->v_x) > 1) { max_friction /= 2; }  // moving: dynamic friction
            double new_diff_v_x = clamp(-max_friction, diff_v_x - node->v_x, max_friction);
            node->v_x += new_diff_v_x - diff_v_x;
            diff_v_x = new_diff_v_x;
            // restitution
            double new_diff_v_y = diff_v_y + _bias - node->v_y;
            node->v_y += new_diff_v_y - diff_v_y;
            diff_v_y = new_diff_v_y;
        }
    }

    CollisionDetector::CollisionDetector(double size_x, double size_y)
        : size_x(size_x), size_y(size_y) {
        _autosize_x = size_x <= 0;
        _autosize_y = size_y <= 0;
    }


    inline void CollisionDetector::add_rect(Rect* rect) {
        rects.push_back(rect);
        _bins.clear();
    }

    inline int CollisionDetector::_bin_x(double x) {
        return floor((x - _min_x_bin) / size_x);
    }
    inline int CollisionDetector::_bin_y(double y) {
        return floor((y - _min_y_bin) / size_y);
    }

    inline void CollisionDetector::_autosize() {
        if (_autosize_x) {
            vector<double> widths;
            for (auto& rect: rects) { widths.push_back(rect->xR - rect->xL); }
            double mean_width = accumulate(widths.begin(), widths.end(), 0.0) / widths.size();
            size_x = 3 * mean_width; // heuristic. TODO: refine.
        }
        if (_autosize_y) {
            vector<double> heights;
            for (auto& rect: rects) { heights.push_back(rect->yT - rect->yB); }
            double mean_height = accumulate(heights.begin(), heights.end(), 0.0) / heights.size();
            size_y = 3 * mean_height; // heuristic. TODO: refine.
        }
    }

    inline void CollisionDetector::_prepare() {
        _autosize();

        vector<double> xLs, xRs, yBs, yTs;
        for (auto& rect: rects) {
            xLs.push_back(rect->xL); xRs.push_back(rect->xR);
            yBs.push_back(rect->yB); yTs.push_back(rect->yT);
        }
        double min_x = *min_element(xLs.begin(), xLs.end());
        double max_x = *max_element(xRs.begin(), xRs.end());
        double min_y = *min_element(yBs.begin(), yBs.end());
        double max_y = *max_element(yTs.begin(), yTs.end());

        _min_x_bin = size_x * floor(min_x / size_x);
        _min_y_bin = size_y * floor(min_y / size_y);
        n_bins_x = floor(max_x / size_x) - floor(min_x / size_x) + 1;
        n_bins_y = floor(max_y / size_y) - floor(min_y / size_y) + 1;
        n_bins = n_bins_x * n_bins_y;

        for (int i = 0; i < n_bins_x; i++) {
            _bins.push_back(vector<vector<Rect*>>());
            for (int j = 0; j < n_bins_y; j++) {
                _bins[i].push_back(vector<Rect*>());
            }
        }

        for (auto& rect: rects) {
            for (int i = _bin_x(rect->xL); i <= _bin_x(rect->xR); i++) {
                for (int j = _bin_y(rect->yB); j <= _bin_y(rect->yT); j++) {
                    _bins[i][j].push_back(rect);
                }
            }
        }
    }

    inline void CollisionDetector::detect_collisions(vector<Node*> &nodes,
             double restitution_threshold, vector<Collision> &collisions) {
        if (rects.size() == 0 || nodes.size() == 0) { return; }
        if (_bins.size() == 0) { _prepare(); }

        for (Node* node: nodes) {
            int bin_x = _bin_x(node->x);
            int bin_y = _bin_y(node->y);
            if (0 <= bin_y && bin_y < n_bins_y && 0 <= bin_x && bin_x < n_bins_x) {
                for (Rect* rect: _bins[bin_x][bin_y]) {
                    if (rect->collides(node)) {
                        Collision col = Collision(rect, node, restitution_threshold);
                        collisions.push_back(col);
                        node->colliding = true;
                        // probably problematic when two or more rectangles overlap and share an edge
                    }
                }
            }
        }
    }
}
