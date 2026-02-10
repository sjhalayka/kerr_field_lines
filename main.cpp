#include "main.h"
#include <thread>
#include <vector>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <algorithm>

// Atomic counter for progress tracking
std::atomic<long long unsigned int> global_progress(0);
const real_type a_star = 0.5;

const real_type angle = pi / 4.0;// 16.0;

vector_3 slerp(vector_3 s0, vector_3 s1, const real_type t)
{
	if (t <= 0.0) return s0;
	if (t >= 1.0) return s1;

	vector_3 s0_norm = s0;
	s0_norm.normalize();

	vector_3 s1_norm = s1;
	s1_norm.normalize();

	real_type cos_angle = s0_norm.dot(s1_norm);
	cos_angle = std::clamp(cos_angle, static_cast<real_type>(-1.0), static_cast<real_type>(1.0));
	const real_type angle = acos(cos_angle);

	// Nearly identical vectors — just return s0
	if (angle < 1e-12)
		return s0;

	const real_type p0_factor = sin((1 - t) * angle) / sin(angle);
	const real_type p1_factor = sin(t * angle) / sin(angle);

	return s0 * p0_factor + s1 * p1_factor;
}

real_type intersect_AABB(const vector_3 min_location, const vector_3 max_location, const vector_3 ray_origin, const vector_3 ray_dir, vector_3 sideways, real_type& tmin, real_type& tmax, const real_type receiver_radius)
{
	// --- X axis ---
	if (fabs(ray_dir.x) < 1e-12)
	{
		// Ray is parallel to X slabs
		if (ray_origin.x < min_location.x || ray_origin.x > max_location.x)
			return 0;
		tmin = -1e30;
		tmax = 1e30;
	}
	else
	{
		tmin = (min_location.x - ray_origin.x) / ray_dir.x;
		tmax = (max_location.x - ray_origin.x) / ray_dir.x;

		if (tmin > tmax)
			swap(tmin, tmax);
	}

	// --- Y axis ---
	real_type tymin, tymax;

	if (fabs(ray_dir.y) < 1e-12)
	{
		// Ray is parallel to Y slabs
		if (ray_origin.y < min_location.y || ray_origin.y > max_location.y)
			return 0;
		tymin = -1e30;
		tymax = 1e30;
	}
	else
	{
		tymin = (min_location.y - ray_origin.y) / ray_dir.y;
		tymax = (max_location.y - ray_origin.y) / ray_dir.y;

		if (tymin > tymax)
			swap(tymin, tymax);
	}

	if ((tmin > tymax) || (tymin > tmax))
		return 0;

	if (tymin > tmin)
		tmin = tymin;

	if (tymax < tmax)
		tmax = tymax;

	// --- Z axis ---
	real_type tzmin, tzmax;

	if (fabs(ray_dir.z) < 1e-12)
	{
		// Ray is parallel to Z slabs
		if (ray_origin.z < min_location.z || ray_origin.z > max_location.z)
			return 0;
		tzmin = -1e30;
		tzmax = 1e30;
	}
	else
	{
		tzmin = (min_location.z - ray_origin.z) / ray_dir.z;
		tzmax = (max_location.z - ray_origin.z) / ray_dir.z;

		if (tzmin > tzmax)
			swap(tzmin, tzmax);
	}

	if ((tmin > tzmax) || (tzmin > tmax))
		return 0;

	if (tzmin > tmin)
		tmin = tzmin;

	if (tzmax < tmax)
		tmax = tzmax;

	if (tmin < 0 || tmax < 0)
		return 0;

	vector_3 ray_hit_start = ray_origin;
	ray_hit_start.x += ray_dir.x * tmin;
	ray_hit_start.y += ray_dir.y * tmin;
	ray_hit_start.z += ray_dir.z * tmin;

	vector_3 ray_hit_end = ray_origin;
	ray_hit_end.x += ray_dir.x * tmax;
	ray_hit_end.y += ray_dir.y * tmax;
	ray_hit_end.z += ray_dir.z * tmax;

	real_type l = (ray_hit_end - ray_hit_start).length();

	//cout << "l " << l << " sideways.length " << sideways.length() << endl;

	return (l);// -l * sideways.length());// ((ray_hit_end - ray_hit_start) * (sideways.length() / l)).length();
}

real_type intersect(
	const vector_3 location,
	const vector_3 normal,
	const vector_3 sideways,
	const vector_3 aabb_min_location,
	const vector_3 aabb_max_location,
	const real_type receiver_radius)
{
	real_type tmin = 0, tmax = 0;

	return intersect_AABB(aabb_min_location, aabb_max_location, location, normal, sideways, tmin, tmax, receiver_radius);

	//vector_3 up_min_location = min_location;
	//up_min_location.y += 2.0 * receiver_radius;

	//vector_3 up_max_location = max_location;
	//up_max_location.y += 2.0 * receiver_radius;

	//vector_3 down_min_location = min_location;
	//down_min_location.y -= 2.0 * receiver_radius;

	//vector_3 down_max_location = max_location;
	//down_max_location.y -= 2.0 * receiver_radius;

	//vector_3 left_min_location = min_location;
	//left_min_location.x += 2.0 * receiver_radius;

	//vector_3 left_max_location = max_location;
	//left_max_location.x += 2.0 * receiver_radius;

	//vector_3 right_min_location = min_location;
	//right_min_location.x -= 2.0 * receiver_radius;

	//vector_3 right_max_location = max_location;
	//right_max_location.x -= 2.0 * receiver_radius;

	//vector_3 forward_min_location = min_location;
	//forward_min_location.z += 2.0 * receiver_radius;

	//vector_3 forward_max_location = max_location;
	//forward_max_location.z += 2.0 * receiver_radius;

	//vector_3 back_min_location = min_location;
	//back_min_location.z -= 2.0 * receiver_radius;

	//vector_3 back_max_location = max_location;
	//back_max_location.z -= 2.0 * receiver_radius;

	//real_type up = intersect_AABB(up_min_location, up_max_location, location, normal, tmin, tmax);
	//real_type down = intersect_AABB(down_min_location, down_max_location, location, normal, tmin, tmax);

	//real_type left = intersect_AABB(left_min_location, left_max_location, location, normal, tmin, tmax);
	//real_type right = intersect_AABB(right_min_location, right_max_location, location, normal, tmin, tmax);

	//real_type forward = intersect_AABB(forward_min_location, forward_max_location, location, normal, tmin, tmax);
	//real_type back = intersect_AABB(back_min_location, back_max_location, location, normal, tmin, tmax);

	//return (forward + center + back) / 3.0;
}

// Thread-local versions of random functions that take generator and distribution as parameters
vector_3 random_cosine_weighted_hemisphere(vector_3 normal,
	std::mt19937& local_gen,
	std::uniform_real_distribution<real_type>& local_dis)
{
	vector_3 r = vector_3(local_dis(local_gen), local_dis(local_gen), 0.0);
	vector_3 uu = normal.cross(vector_3(0.0, 1.0, 1.0)).normalize();
	vector_3 vv = uu.cross(normal);

	double ra = sqrt(r.y);
	double rx = ra * cos(2.0 * pi * r.x);
	double ry = ra * sin(2.0 * pi * r.x);
	double rz = sqrt(1.0 - r.y);
	vector_3 rr = vector_3(uu * rx + vv * ry + normal * rz);

	return rr.normalize();
}

vector_3 random_unit_vector(std::mt19937& local_gen,
	std::uniform_real_distribution<real_type>& local_dis)
{
	const real_type z = local_dis(local_gen) * 2.0 - 1.0;
	const real_type a = local_dis(local_gen) * 2.0 * pi;

	const real_type r = sqrt(1.0f - z * z);
	const real_type x = r * cos(a);
	const real_type y = r * sin(a);

	return vector_3(x, y, z).normalize();
}


vector_3 cartesianToSpherical(vector_3 input)
{
	vector_3 s;
	s.z = std::sqrt(input.x * input.x + input.y * input.y + input.z * input.z); // distance (radius) - calculate first!
	s.x = std::acos(input.z / s.z);          // polar angle (theta/colatitude)
	s.y = std::atan2(input.y, input.x);              // azimuthal angle (phi)

	return s;
}




// Worker function for each thread
void worker_thread(
	long long unsigned int start_idx,
	long long unsigned int end_idx,
	unsigned int thread_seed,
	const real_type emitter_radius,
	const real_type emitter_mass,
	const real_type receiver_distance,
	const real_type receiver_distance_plus,
	const real_type receiver_radius,
	const real_type epsilon,
	real_type& result_count,
	real_type& result_count_plus)
{
	// Thread-local random number generator
	std::mt19937 local_gen(thread_seed);
	std::uniform_real_distribution<real_type> local_dis(0.0, 1.0);

	real_type local_count = 0;
	real_type local_count_plus = 0;

	// Update progress every N iterations to reduce atomic overhead
	const long long unsigned int progress_update_interval = 10000;
	long long unsigned int local_progress = 0;

	const vector_3 up(0, 1, 0);

	for (long long unsigned int i = start_idx; i < end_idx; i++)
	{
		vector_3 location = random_unit_vector(local_gen, local_dis) * emitter_radius;
		vector_3 r = random_unit_vector(local_gen, local_dis) * emitter_radius;

		const vector_3 pre_rotate_normal = (location - r).normalize();

		location.rotate_z(-pi / 2.0);
		r.rotate_z(-pi / 2.0);
		location.rotate_z(angle);
		r.rotate_z(angle);

		const vector_3 normal = (location - r).normalize();
		const vector_3 spherical = cartesianToSpherical(normal);

		vector_3 sideways = normal.cross(up);


		real_type sideways_length = a_star * sin(spherical.x) / (1 + sqrt(1 - a_star * a_star));

		//cout << receiver_radius << endl;
		//cout << receiver_distance << endl;
		//cout << sigma << endl;
		//cout << endl;


		if (1)//local_dis(local_gen) > (1 - dt_div_dtau)) //(a_star * a_star) * abs(pre_rotation_normal.dot(up)))
			//if (local_dis(local_gen) > abs(pre_rotation_normal.y))
		{

			//vector_3 p_disk = normal;
			//p_disk.y = 0;

			//// Guard against near-zero p_disk (happens when normal is near-vertical)
			//real_type p_disk_len = sqrt(p_disk.x * p_disk.x + p_disk.y * p_disk.y + p_disk.z * p_disk.z);

			//if (p_disk_len < 1e-12)
			//{
			//	// normal is nearly vertical, skip slerp — use normal as-is
			//	// (p_disk is degenerate, slerp would produce NaN)
			//}
			//else
			//{
			//	//p_disk.normalize();
			//	//normal = slerp(normal, p_disk, a_star);
			//}



			sideways.normalize();
			sideways *= sideways_length;

			vector_3 aabb_min_location(-receiver_radius + receiver_distance, -receiver_radius, -receiver_radius);
			vector_3 aabb_max_location(receiver_radius + receiver_distance, receiver_radius, receiver_radius);

			vector_3 right_min_location = aabb_min_location;
			right_min_location.x += epsilon;

			vector_3 right_max_location = aabb_max_location;
			right_max_location.x += epsilon;

			vector_3 left_min_location = aabb_min_location;
			left_min_location.x -= epsilon;

			vector_3 left_max_location = aabb_max_location;
			left_max_location.x -= epsilon;


			const real_type a = a_star * emitter_mass;
			const real_type b =
				receiver_distance * receiver_distance
				+ a * a * pow(cos(angle), 2.0);

			const real_type dt_Kerr = sqrt(1 - emitter_radius * receiver_distance / b);
			const real_type a_Kerr_geometrized =
				emitter_radius / (pi * b * dt_Kerr);

			const real_type fractionality = 1.0 - 2.0 * (0.5 - fmod(a_star, 1.0));

			local_count += intersect(
				location, normal, sideways,
				aabb_min_location, aabb_max_location,
				receiver_radius) / ((1.0 / (1.0 + fractionality)) * (1 + (1.0 / (2 * fractionality)) + 4 * pi * a_star * cos(angle) * cos(angle)));

			local_count_plus += intersect(
				location, normal, sideways,
				right_min_location, right_max_location,
				receiver_radius) / ((1.0/(1.0 + fractionality))*(1 + (1.0 /(2 * fractionality)) + 4 * pi * a_star * cos(angle) * cos(angle)));
		
			//local_count *= 1.0 + fractionality;
			//local_count_plus *= 1.0 + fractionality;
		}

		// Update global progress periodically
		local_progress++;
		if (local_progress >= progress_update_interval)
		{
			global_progress.fetch_add(local_progress, std::memory_order_relaxed);
			local_progress = 0;
		}
	}

	// Add any remaining progress
	if (local_progress > 0)
	{
		global_progress.fetch_add(local_progress, std::memory_order_relaxed);
	}

	result_count = local_count;
	result_count_plus = local_count_plus;
}

// Progress monitor function that runs on main thread
void progress_monitor(long long unsigned int total_iterations, std::atomic<bool>& done)
{
	auto start_time = std::chrono::steady_clock::now();

	while (!done.load(std::memory_order_relaxed))
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		long long unsigned int current = global_progress.load(std::memory_order_relaxed);
		double progress = static_cast<double>(current) / static_cast<double>(total_iterations);

		auto now = std::chrono::steady_clock::now();
		auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();

		// Estimate time remaining
		double eta_seconds = 0;
		if (progress > 0.001)
		{
			eta_seconds = (elapsed / progress) * (1.0 - progress);
		}

		cout << "\rProgress: " << fixed << (progress * 100.0) << "% "
			<< "| Elapsed: " << elapsed << "s "
			<< "| ETA: " << static_cast<int>(eta_seconds) << "s    " << flush;
	}

	cout << "\rProgress: 100.00% | Complete!                              " << endl;
}

real_type get_intersecting_line_density(
	const long long unsigned int n,
	const real_type emitter_radius,
	const real_type emitter_mass,
	const real_type receiver_distance,
	const real_type receiver_distance_plus,
	const real_type receiver_radius,
	const real_type epsilon)
{
	// Reset global progress counter
	global_progress.store(0, std::memory_order_relaxed);

	// Get number of hardware threads
	unsigned int num_threads = std::thread::hardware_concurrency();
	cout << "Using " << num_threads << " threads for " << n << " iterations" << endl;

	std::vector<std::thread> threads;
	std::vector<real_type> thread_counts(num_threads, 0);
	std::vector<real_type> thread_counts_plus(num_threads, 0);

	// Flag to signal progress monitor to stop
	std::atomic<bool> done(false);

	// Start progress monitor thread
	std::thread monitor_thread(progress_monitor, n, std::ref(done));

	// Calculate work distribution
	long long unsigned int iterations_per_thread = n / num_threads;
	long long unsigned int remainder = n % num_threads;

	long long unsigned int current_start = 0;

	for (unsigned int t = 0; t < num_threads; t++)
	{
		long long unsigned int thread_iterations = iterations_per_thread;
		if (t < remainder) thread_iterations++; // Distribute remainder

		long long unsigned int thread_end = current_start + thread_iterations;

		// Each thread gets a different seed based on thread index
		unsigned int thread_seed = t + static_cast<unsigned>(time(0));

		threads.emplace_back(
			worker_thread,
			current_start,
			thread_end,
			thread_seed,
			emitter_radius,
			emitter_mass,
			receiver_distance,
			receiver_distance_plus,
			receiver_radius,
			epsilon,
			std::ref(thread_counts[t]),
			std::ref(thread_counts_plus[t])
		);

		current_start = thread_end;
	}

	// Wait for all worker threads to complete
	for (auto& t : threads)
	{
		t.join();
	}

	// Signal monitor thread to stop and wait for it
	done.store(true, std::memory_order_relaxed);
	monitor_thread.join();

	// Aggregate results
	real_type total_count = 0;
	real_type total_count_plus = 0;

	for (unsigned int t = 0; t < num_threads; t++)
	{
		total_count += thread_counts[t];
		total_count_plus += thread_counts_plus[t];
	}

	return total_count_plus - total_count;
}

int main(int argc, char** argv)
{
	ofstream outfile_numerical("Kerr_numerical");
	ofstream outfile_analytical("Kerr_analytical");
	ofstream outfile_Newton("Newton_analytical");

	// Field line count
	const real_type n = 1e9;

	const real_type emitter_mass_geometrized =
		sqrt((n * log(2.0)) / (2 * pi * (1 + sqrt(1 - a_star * a_star))));

	const real_type emitter_radius_geometrized =
		emitter_mass_geometrized * (1 + sqrt(1 - a_star * a_star));

	const real_type receiver_radius_geometrized =
		emitter_radius_geometrized * 0.01; // Minimum one Planck unit

	const real_type emitter_area_geometrized =
		4 * pi
		* (emitter_radius_geometrized * emitter_radius_geometrized
			+ a_star * a_star
			* emitter_mass_geometrized * emitter_mass_geometrized);




	real_type start_pos =
		emitter_radius_geometrized
		+ receiver_radius_geometrized;

	real_type end_pos = start_pos * 10.0;

	const size_t pos_res = 20; // Minimum 2 steps

	const real_type pos_step_size =
		(end_pos - start_pos)
		/ (pos_res - 1);

	const real_type epsilon =
		receiver_radius_geometrized;


	for (size_t i = 0; i < pos_res; i++)
	{
		cout << "\n=== Step " << (i + 1) << " of " << pos_res << " ===" << endl;

		const real_type receiver_distance_geometrized =
			start_pos + i * pos_step_size;

		const real_type receiver_distance_plus_geometrized =
			receiver_distance_geometrized + epsilon;

		// beta function
		const real_type collision_count_plus_minus_collision_count =
			get_intersecting_line_density(
				static_cast<long long unsigned int>(n),
				emitter_radius_geometrized,
				emitter_mass_geometrized,
				receiver_distance_geometrized,
				receiver_distance_plus_geometrized,
				receiver_radius_geometrized,
				epsilon);

		// alpha variable
		const real_type gradient_integer =
			collision_count_plus_minus_collision_count
			/ epsilon;

		// g variable
		real_type gradient_strength =
			-gradient_integer
			/
			(2.0 * receiver_radius_geometrized
				* receiver_radius_geometrized
				* receiver_radius_geometrized);

		const real_type a_Newton_geometrized =
			sqrt(
				n * log(2.0)
				/
				(4.0 * pi *
					pow(receiver_distance_geometrized, 4.0))
			);

		const real_type dt_Schwarzschild = sqrt(1 - emitter_radius_geometrized / receiver_distance_geometrized);

		const real_type a_Schwarzschild_geometrized =
			emitter_radius_geometrized / (pi * pow(receiver_distance_geometrized, 2.0) * dt_Schwarzschild);

		const real_type a_flat_geometrized =
			gradient_strength * receiver_distance_geometrized * log(2)
			/ (8.0 * emitter_mass_geometrized);

		const real_type a = a_star * emitter_mass_geometrized;
		const real_type b =
			receiver_distance_geometrized * receiver_distance_geometrized
			+ a * a * pow(cos(angle), 2.0);

		const real_type dt_Kerr = sqrt(1 - emitter_radius_geometrized * receiver_distance_geometrized / b);
		const real_type a_Kerr_geometrized =
			emitter_radius_geometrized / (pi * b * dt_Kerr);

		cout << "a_Schwarzschild_geometrized " << a_Schwarzschild_geometrized << endl;
		cout << "a_Kerr_geometrized " << a_Kerr_geometrized << endl;
		cout << "a_Newton_geometrized " << a_Newton_geometrized << endl;
		cout << "a_flat_geometrized " << a_flat_geometrized << endl;
		cout << a_Kerr_geometrized / a_flat_geometrized << endl;

		outfile_numerical << receiver_distance_geometrized << " " << a_flat_geometrized << endl;
		outfile_analytical << receiver_distance_geometrized << " " << a_Kerr_geometrized << endl;
		outfile_Newton << receiver_distance_geometrized << " " << a_Newton_geometrized << endl;
	}

}