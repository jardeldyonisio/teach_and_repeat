import bezier
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist

from poplt import poplt
from segop import segop
from knots import knots
from globop import globop
from cpoints import cpoints
from iguess0 import iguess0
from DrawBezierCurve import drawBezierCurve
from cubicBezierToPolyline import cubicBezierToPolyline


class BsplieFit:
    '''
    @brief Class for fitting Piecewise G1 Bezier curves to trajectory data.
    
    This class implements the Lane 1995 algorithm with adaptive knot placement
    for fitting cubic Bezier curves with G1 continuity to robot trajectory data.
    '''
    
    def __init__(self):
        '''
        @brief Initialize the BsplieFit with default values.
        '''
        # Results storage
        self.new_control_points = None     # Cnew - control points array (Nx2)
        self.knot_positions = None     # P - spatial positions of knots (Nx2)
        self.knot_indices = None       # k - indices of knots in trajectory (1-based)
        self.bezier_points = None      # Q - original trajectory points (Nx2)
        self.best_error = None         # Error from best iteration
        self.error_history = None      # History of errors during adaptive fitting
        self.iteration_info = None     # Detailed iteration information
        self.GOC = None               # Global Optimized Curve result
        
        # Internal variables
        self._fitted = False          # Flag to track if fitting has been performed
    
    def getControlPoints(self):
        '''
        @brief Retrieve control points from the Bezier curve fitting process.
        @return Control points array (Nx2) or None if not fitted
        '''
        if not self._fitted:
            print('Warning: No control points available. Run fit() first.')
            return None
        return self.new_control_points
    
    def getKnotPositions(self):
        '''
        @brief Retrieve spatial positions of knot points.
        @return Knot positions array (Nx2) with x,y coordinates or None if not fitted
        '''
        if not self._fitted:
            print('Warning: No knot positions available. Run fit() first.')
            return None
        return self.knot_positions
    
    def getKnotIndices(self):
        '''
        @brief Retrieve indices of knot points in the original trajectory.
        @return Knot indices array (1-based for MATLAB compatibility) or None if not fitted
        '''
        if not self._fitted:
            print('Warning: No knot indices available. Run fit() first.')
            return None
        return self.knot_indices
    
    def getBezierPoints(self):
        '''
        @brief Retrieve original trajectory points.
        @return Original trajectory points array (Nx2) or None if not fitted
        '''
        if not self._fitted:
            print('Warning: No bezier points available. Run fit() first.')
            return None
        return self.bezier_points
    
    def getError(self):
        '''
        @brief Retrieve error metrics from the best iteration.
        @return Best error value or None if not fitted
        '''
        if not self._fitted:
            print('Warning: No error data available. Run fit() first.')
            return None
        return self.best_error
    
    def fit(self, 
            control_points=None, 
            data_points=None, 
            adaptive=True,
            initial_knots=3,
            max_knots=20,
            error_threshold=0.2,
            max_iterations=10):
        '''
        @brief Fit Bezier curves to trajectory data.
        
        @param control_points Control points for generating trajectory (Nx2 array)
        @param data_points Trajectory data points to fit (Nx2 array)
        @param adaptive Whether to use adaptive knot placement
        @param initial_knots Initial number of knots for adaptive algorithm
        @param max_knots Maximum number of knots for adaptive algorithm
        @param error_threshold Target error threshold for convergence
        @param max_iterations Maximum iterations for adaptive algorithm
        @return True if fitting successful, False otherwise
        '''
        try:
            # Determine input data
            if control_points is not None and data_points is not None:
                raise ValueError('Provide either control_points OR data_points, not both.')
            
            if control_points is not None:
                # Generate trajectory from control points
                Q = cubicBezierToPolyline(control_points, 65)
                print('Generated trajectory from control points')
            elif data_points is not None:
                # Use provided trajectory data
                Q = np.array(data_points)
                print('Using provided trajectory data points')
            else:
                # Generate default spiral trajectory for demonstration
                t = np.linspace(0, 4*np.pi, 200)
                np.random.seed(42)  # For reproducible results
                noise_x = np.random.normal(0, 0.02, 200)
                noise_y = np.random.normal(0, 0.02, 200)
                x = t * np.cos(t) * 0.3 + noise_x
                y = t * np.sin(t) * 0.3 + noise_y
                Q = np.column_stack([x, y])
                print('Generated default spiral trajectory for demonstration')
            
            # Store original trajectory points
            self.bezier_points = Q.copy()
            
            # Determine knot placement strategy
            if adaptive:
                # Use adaptive knot placement algorithm
                k_adaptive, error_history, iteration_info = self._adaptiveKnotsPlacement(
                    Q, 
                    initial_knots=initial_knots,
                    max_knots=max_knots,
                    error_threshold=error_threshold,
                    max_iterations=max_iterations
                )
                
                self.knot_indices = k_adaptive
                self.error_history = error_history
                self.iteration_info = iteration_info
                n = len(k_adaptive)
                
                # Get best error from final iteration
                if error_history:
                    self.best_error = error_history[-1]
                
            else:
                # Use uniform knot distribution
                n = initial_knots
                self.knot_indices = np.linspace(1, len(Q), n).astype(int)
                self.error_history = []
                self.iteration_info = []
                print(f'Using uniform knot distribution with {n} knots')
            
            # Run final optimization with chosen knots
            Qt = Q.T
            IG, k_out, dpkpc = iguess0(Qt, n, self.knot_indices)
            
            # Improve the fit: Segmentally Optimum Only Curve (SOO)
            SOC = segop(self.knot_indices, Qt, IG)
            
            # Improve it again: Segmentally then Globally Optimized Curve (SGO)
            self.GOC = globop(SOC, Qt, 0, self.knot_indices, dpkpc)
            
            # Extract final results
            self.new_control_points = cpoints(self.GOC).T  # Transpose to Nx2
            self.knot_positions = knots(Qt, self.knot_indices).T  # Transpose to Nx2
            
            # Calculate final error if not adaptive
            if not adaptive:
                segment_errors = self._calculateSegmentErrors(self.GOC, Qt, self.knot_indices, dpkpc)
                self.best_error = np.sum(segment_errors)
            
            self._fitted = True
            
            return True
            
        except Exception as e:
            print(f'Error during fitting: {str(e)}')
            self._fitted = False
            return False

    def _generateCubicBezierCurve(self, 
                                  control_points,
                                  num_points_per_segment=100):
        '''
        @brief Generate a piecewise Bezier curve from control points.

        @param control_points Control points for all Bezier segments (Nx2)
        @param num_points_per_segment Number of points to generate per segment
        @return Generated Bezier curve points (Mx2)
        '''
        # TODO: Implement 'num_points_per_segment' based on the segment length and characteristics.

        try:
            if control_points is None or len(control_points) < 4:
                raise ValueError("Need at least 4 control points for a cubic Bezier curve")
            
            all_curve_points = []
            
            # Calculate number of segments for piecewise Bezier curves
            # Pattern: [P0,P1,P2,P3] for first segment, then [P3,P4,P5,P6] for second, etc.
            # So: total_points = 1 + 3*num_segments
            # Therefore: num_segments = (total_points - 1) / 3
            total_control_points = len(control_points)
            num_segments = (total_control_points - 1) // 3
            
            if num_segments == 0:
                # Only one segment possible
                num_segments = 1
            
            print(f"Generating {num_segments} Bezier segments from {total_control_points} control points")
            
            for segment in range(num_segments):
                # Get 4 control points for this segment
                # First segment: points 0,1,2,3
                # Second segment: points 3,4,5,6 (sharing point 3)
                # Third segment: points 6,7,8,9 (sharing point 6)
                start_idx = segment * 3
                end_idx = start_idx + 4
                
                if end_idx <= total_control_points:
                    segment_control_points = control_points[start_idx:end_idx]
                    
                    # Convert to format expected by bezier library (2xN array)
                    nodes = np.asfortranarray([
                        segment_control_points[:, 0],  # x coordinates
                        segment_control_points[:, 1],  # y coordinates
                    ])
                    
                    # Create Bezier curve object
                    curve = bezier.Curve(nodes, degree=3)
                    
                    # Generate points on this segment
                    # For continuity, skip first point of subsequent segments
                    if segment == 0:
                        t_vals = np.linspace(0.0, 1.0, num_points_per_segment)
                    else:
                        # Skip first point to avoid duplication
                        t_vals = np.linspace(0.0, 1.0, num_points_per_segment)[1:]  
                    
                    # Evaluate curve at parameter values
                    segment_points = curve.evaluate_multi(t_vals)
                    
                    # Convert back to Nx2 format and add to result
                    segment_points_2d = segment_points.T  # Transpose to get (num_points, 2)
                    all_curve_points.append(segment_points_2d)
                    
                    print(f"  Segment {segment + 1}: Generated {len(segment_points_2d)} points")
                else:
                    print(f"  Warning: Not enough control points for segment {segment + 1}")
                    break
            
            if not all_curve_points:
                raise ValueError("No curve segments could be generated")
            
            # Concatenate all segments
            complete_curve = np.vstack(all_curve_points)

            print(f"Generated complete piecewise Bezier curve with {len(complete_curve)} points")
            return complete_curve
            
        except Exception as e:
            print(f"Error generating Bezier curve: {str(e)}")
            # Fallback: return original control points if generation fails
            return control_points

    def _adaptiveKnotsPlacement(self, 
                                Q,
                                initial_knots=3, 
                                max_knots=20, 
                                error_threshold=0.2, 
                                max_iterations=10):
        '''
        @brief Adaptive knot placement algorithm that iteratively adds knots where error is highest.
        
        @param Q trajectory points (Nx2)
        @param initial_knots starting number of knots
        @param max_knots maximum number of knots allowed
        @param error_threshold target error threshold to stop iteration
        @param max_iterations maximum number of iterations to prevent infinite loops
        
        @return best_k optimal knot indices (1-based for MATLAB compatibility)
        @return error_history history of errors for analysis
        @return iteration_info detailed information about each iteration
        '''
        
        Qt = Q.T  # Transpose for algorithm compatibility
        error_history = []
        iteration_info = []
        
        # Start with uniform distribution
        current_knots = np.linspace(1, len(Q), initial_knots).astype(int)
        
        for iteration in range(max_iterations):
            try:
                # Run full Bezier fitting pipeline
                IG, k_out, dpkpc = iguess0(Qt, len(current_knots), current_knots)
                SOC = segop(current_knots, Qt, IG)
                GOC = globop(SOC, Qt, 0, current_knots, dpkpc)
                
                # Calculate error for each segment
                segment_errors = self._calculateSegmentErrors(GOC, Qt, current_knots, dpkpc)
                total_error = np.sum(segment_errors)
                
                print(f'Total error: {total_error:.6f}')
                print(f'Segment errors: {segment_errors}')
                
                error_history.append(total_error)
                iteration_info.append({
                    'iteration': iteration + 1,
                    'knots': current_knots.copy(),
                    'total_error': total_error,
                    'segment_errors': segment_errors.copy(),
                    'GOC': GOC.copy()
                })
                
                # Check convergence
                if total_error < error_threshold:
                    print(f'Converged! Error {total_error:.6f} < threshold {error_threshold}')
                    break
                    
                if len(current_knots) >= max_knots:
                    print(f'Reached maximum knots ({max_knots})')
                    break
                
                # Find segment with highest error and add knot in middle
                worst_segment = np.argmax(segment_errors)
                
                # Calculate position for new knot (middle of worst segment)
                if worst_segment < len(current_knots) - 1:
                    start_idx = current_knots[worst_segment] - 1  # Convert to 0-based
                    end_idx = current_knots[worst_segment + 1] - 1
                    new_knot_idx = (start_idx + end_idx) // 2 + 1  # Convert back to 1-based
                    
                    # Insert new knot
                    insert_pos = worst_segment + 1
                    current_knots = np.insert(current_knots, insert_pos, new_knot_idx)
                    
                    print(f'Added knot at index {new_knot_idx} (segment {worst_segment} had error {segment_errors[worst_segment]:.6f})')
                else:
                    print('Cannot add more knots - reached trajectory end')
                    break
                    
            except Exception as e:
                print(f'Error in iteration {iteration + 1}: {str(e)}')
                if iteration == 0:
                    # If first iteration fails, fall back to simple uniform distribution
                    current_knots = np.linspace(1, len(Q), min(8, max_knots)).astype(int)
                    print(f'Falling back to uniform distribution: {current_knots}')
                break
        
        print(f'Final error: {error_history[-1] if error_history else "N/A"}')
        print(f'Iterations completed: {len(error_history)}')
        
        return current_knots, error_history, iteration_info
    
    def _calculateSegmentErrors(self, GOC, Q, k, dpkpc):
        '''
        @brief Calculate the error for each Bezier segment.

        @param GOC: Global control points
        @param Q: Data points
        @param k: Knot vector
        @param dpkpc: Derivative control points
        @return Array of segment errors
        '''
        try:
            # Get control points
            C = cpoints(GOC)
            
            # Calculate error for each segment
            segment_errors = []
            
            for i in range(len(k) - 1):
                # Get data points for this segment
                start_idx = k[i] - 1  # Convert to 0-based
                end_idx = k[i + 1] - 1
                
                # Get control points for this segment (4 points per cubic Bezier)
                ctrl_start = i * 3
                if ctrl_start + 3 < C.shape[1]:
                    ctrl_points = C[:, ctrl_start:ctrl_start + 4]
                    segment_data = Q[:, start_idx:end_idx + 1]
                    
                    # Calculate sum of distances for this segment
                    if segment_data.shape[1] > 0:
                        error = self._calculateSegmentDistanceError(ctrl_points, segment_data)
                        segment_errors.append(error)
                    else:
                        segment_errors.append(0.0)
                else:
                    segment_errors.append(0.0)
            
            return np.array(segment_errors)
            
        except Exception as e:
            print(f'Error calculating segment errors: {str(e)}')
            return np.array([1.0] * (len(k) - 1))  # Return uniform error as fallback

    def _calculateSegmentDistanceError(self, ctrl_points, data_points):
        '''
        @brief Calculate the sum of minimum distances from data points to Bezier curve segment.

        @param ctrl_points: Control points of the Bezier curve segment
        @param data_points: Data points to measure distances from

        @return Sum of minimum distances
        '''
        try:
            # Generate points on the Bezier curve
            t_vals = np.linspace(0, 1, 50)
            bezier_points = []
            
            for t in t_vals:
                # Cubic Bezier formula: B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃
                b = ((1-t)**3 * ctrl_points[:, 0:1] + 
                     3*(1-t)**2*t * ctrl_points[:, 1:2] + 
                     3*(1-t)*t**2 * ctrl_points[:, 2:3] + 
                     t**3 * ctrl_points[:, 3:4])
                bezier_points.append(b.flatten())
            
            bezier_points = np.array(bezier_points)  # Shape: (50, 2)
            
            # Calculate minimum distances from each data point to Bezier curve
            distances = []
            for i in range(data_points.shape[1]):
                point = data_points[:, i].reshape(1, -1)  # Shape: (1, 2)
                dists = cdist(point, bezier_points)
                min_dist = np.min(dists)
                distances.append(min_dist)
            
            return np.sum(distances)
            
        except Exception as e:
            print(f'Error in distance calculation: {str(e)}')
            return 1.0  # Return default error
    
    def plot(self):
        '''
        @brief Generate comprehensive analysis plots of the Bezier fitting results.
        
        Creates multiple subplots showing:
        - Basic curve fit with control points
        - Adaptive algorithm convergence (if applicable)
        - Knot evolution analysis
        - Segment error analysis
        - Iteration comparison
        '''
        if not self._fitted:
            print('Warning: No data to plot. Run fit() first.')
            return
        
        Q = self.bezier_points
        
        # Plot basic SGO curve
        plt.figure(figsize=(10, 8))
        Qt = Q.T
        poplt(self.GOC, Qt)
        plt.title('Python: Plot of SGO curve')
        
        # Plot detailed curve fit
        plt.figure(figsize=(12, 8))
        for i in range(0, len(self.new_control_points)-2, 3):
            if i+3 < len(self.new_control_points):
                drawBezierCurve(self.new_control_points[i:i+4])   # Fitted cubic Bézier segment

        plt.plot(self.new_control_points[:,0], self.new_control_points[:,1], 'o-', 
                label='Control points', linewidth=2, markersize=6)
        plt.plot(Q[:,0], Q[:,1], 'k.', label='Original data', markersize=3)
        plt.plot(self.knot_positions[:,0], self.knot_positions[:,1], 'kx', 
                markersize=10, markeredgewidth=3, label=f'Knots (n = {len(self.knot_indices)})')

        plt.legend()
        plt.title('Python: Detailed Bézier Curve Fit')
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        # Advanced analysis plots (only if adaptive was used)
        if self.error_history and self.iteration_info:
            plt.figure(figsize=(15, 10))
            
            # Top subplot: Error convergence
            plt.subplot(2, 3, 1)
            plt.plot(range(1, len(self.error_history) + 1), self.error_history, 
                    'bo-', linewidth=2, markersize=8)
            plt.xlabel('Iteration')
            plt.ylabel('Total Error')
            plt.title('Adaptive Algorithm Convergence')
            plt.grid(True, alpha=0.3)
            
            # Middle subplot: Number of knots evolution
            plt.subplot(2, 3, 2)
            knot_counts = [len(info['knots']) for info in self.iteration_info]
            plt.plot(range(1, len(knot_counts) + 1), knot_counts, 'go-', linewidth=2, markersize=8)
            plt.xlabel('Iteration')
            plt.ylabel('Number of Knots')
            plt.title('Knot Count Evolution')
            plt.grid(True, alpha=0.3)
            
            # Right subplot: Final trajectory with knot evolution
            plt.subplot(2, 3, 3)
            plt.plot(Q[:,0], Q[:,1], 'k-', alpha=0.5, linewidth=1, label='Original trajectory')
            
            # Show knot evolution with different colors
            colors = plt.cm.viridis(np.linspace(0, 1, len(self.iteration_info)))
            for i, info in enumerate(self.iteration_info):
                knot_positions = info['knots'] - 1  # Convert to 0-based
                plt.scatter(Q[knot_positions, 0], Q[knot_positions, 1], 
                           c=[colors[i]], s=50, alpha=0.7, 
                           label=f'Iter {i+1} (n={len(info["knots"])})')
            
            plt.title('Knot Placement Evolution')
            plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
            plt.grid(True, alpha=0.3)
            plt.axis('equal')
            
            # Bottom subplots: Segment error analysis
            plt.subplot(2, 3, 4)
            final_info = self.iteration_info[-1]
            segment_errors = final_info['segment_errors']
            segments = range(1, len(segment_errors) + 1)
            bars = plt.bar(segments, segment_errors, alpha=0.7, color='orange')
            plt.xlabel('Segment Number')
            plt.ylabel('Segment Error')
            plt.title('Final Segment Errors')
            plt.grid(True, alpha=0.3)
            
            # Highlight the worst segment
            if len(segment_errors) > 0:
                worst_idx = np.argmax(segment_errors)
                bars[worst_idx].set_color('red')
            
            # Bottom middle: Comparison of iterations
            plt.subplot(2, 3, 5)
            if len(self.iteration_info) >= 2:
                first_iter = self.iteration_info[0]
                last_iter = self.iteration_info[-1]
                
                categories = ['Initial', 'Final']
                errors = [first_iter['total_error'], last_iter['total_error']]
                knot_counts = [len(first_iter['knots']), len(last_iter['knots'])]
                
                x = np.arange(len(categories))
                width = 0.35
                
                ax1 = plt.gca()
                ax2 = ax1.twinx()
                
                bars1 = ax1.bar(x - width/2, errors, width, alpha=0.7, color='blue', label='Error')
                bars2 = ax2.bar(x + width/2, knot_counts, width, alpha=0.7, color='green', label='Knots')
                
                ax1.set_xlabel('Algorithm Stage')
                ax1.set_ylabel('Total Error', color='blue')
                ax2.set_ylabel('Number of Knots', color='green')
                ax1.set_title('Initial vs Final Comparison')
                ax1.set_xticks(x)
                ax1.set_xticklabels(categories)
                
                # Add value labels on bars
                for bar, value in zip(bars1, errors):
                    ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(errors)*0.02,
                            f'{value:.4f}', ha='center', va='bottom')
                for bar, value in zip(bars2, knot_counts):
                    ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + max(knot_counts)*0.02,
                            f'{value}', ha='center', va='bottom')
            
            # Bottom right: Algorithm summary
            plt.subplot(2, 3, 6)
            plt.axis('off')
            
            improvement = ((self.iteration_info[0]['total_error'] - self.iteration_info[-1]['total_error']) 
                          / self.iteration_info[0]['total_error'] * 100)
            status = 'Converged' if self.error_history[-1] < 0.01 else 'Max iterations reached'
            
            summary_text = f'''
ADAPTIVE ALGORITHM SUMMARY

Initial knots: {self.iteration_info[0]['knots']}
Final knots: {self.iteration_info[-1]['knots']}

Initial error: {self.iteration_info[0]['total_error']:.6f}
Final error: {self.iteration_info[-1]['total_error']:.6f}

Iterations: {len(self.iteration_info)}
Improvement: {improvement:.1f}%

Status: {status}
            '''
            
            plt.text(0.05, 0.95, summary_text, transform=plt.gca().transAxes, 
                     fontsize=10, verticalalignment='top', fontfamily='monospace',
                     bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
            
            plt.tight_layout()
        
        plt.show()

if __name__ == '__main__':
    fitter = BsplieFit()
    success = fitter.fit(adaptive=True, 
                         max_knots=15, 
                         error_threshold=0.01)
    fitter._generateCubicBezierCurve(fitter.new_control_points)

    if success:      
        fitter.plot()
