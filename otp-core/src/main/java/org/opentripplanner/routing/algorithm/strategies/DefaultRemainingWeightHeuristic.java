/* This program is free software: you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public License
 as published by the Free Software Foundation, either version 3 of
 the License, or (props, at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>. */

package org.opentripplanner.routing.algorithm.strategies;

import org.opentripplanner.common.geometry.DistanceLibrary;
import org.opentripplanner.common.geometry.SphericalDistanceLibrary;
import org.opentripplanner.routing.core.OptimizeType;
import org.opentripplanner.routing.core.State;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.vertextype.IntersectionVertex;

/**
 * A Euclidean remaining weight strategy that takes into account transit boarding costs where applicable.
 * 
 */
public class DefaultRemainingWeightHeuristic implements RemainingWeightHeuristic {

    private static final long serialVersionUID = -5172878150967231550L;

    private RoutingRequest options;

    private boolean useTransit = false;

    private double maxSpeed;

    private DistanceLibrary distanceLibrary = SphericalDistanceLibrary.getInstance();

    private TransitLocalStreetService localStreetService;

    private double targetX;

    private double targetY;

    @Override
    public void initialize(State s, Vertex target) {
        this.options = s.getOptions();
        this.useTransit = options.getModes().isTransit();
        this.maxSpeed = getMaxSpeed(options);

        Graph graph = options.rctx.graph;
        localStreetService = graph.getService(TransitLocalStreetService.class);

        targetX = target.getX();
        targetY = target.getY();
    }

    @Override
    public double computeForwardWeight(State s, Vertex target) {
        Vertex sv = s.getVertex();
        double euclideanDistance = distanceLibrary.fastDistance(sv.getY(), sv.getX(), targetY,
                targetX);
        return options.walkReluctance * euclideanDistance / maxSpeed;
    }

    /**
     * computeForwardWeight and computeReverseWeight were identical (except that 
     * computeReverseWeight did not have the localStreetService clause). They have been merged.
     */
    @Override
    public double computeReverseWeight(State s, Vertex target) {
        return computeForwardWeight(s, target);
    }

    /** 
     * Get the maximum expected speed over all modes. This should probably be moved to
     * RoutingRequest. 
     */
    public static double getMaxSpeed(RoutingRequest options) {
        if (options.getModes().contains(TraverseMode.TRANSIT)) {
            // assume that the max average transit speed over a hop is 10 m/s, which is roughly
            // true in Portland and NYC, but *not* true on highways
            return 10;
        } else {
            if (options.optimize == OptimizeType.QUICK) {
                return options.getStreetSpeedUpperBound();
            } else {
                // assume that the best route is no more than 10 times better than
                // the as-the-crow-flies flat base route.
                return options.getStreetSpeedUpperBound() * 10;
            }
        }
    }

    @Override
    public void reset() {}

    @Override
    public void doSomeWork() {}

}
