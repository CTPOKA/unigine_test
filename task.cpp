#include "task.h"
#include <math.h>
#include <thread>
#include <algorithm>

struct tree
{
	struct node
	{
		float min_x, min_y;
		float max_x, max_y;
		uint16_t count;
	};

	std::vector<node> nodes;

	tree(std::vector<vec2>& points) {
		uint16_t n = points.size();
		uint16_t n2 = 2 << (int)log2(n - 1);
		nodes.resize((n2 << 1) - 1);

		vec2* ptr = &points[0];
		minmax(ptr, ptr + n, nodes[0]);
		nodes[0].count = n;

		bool useX = true;
		uint16_t l, r = 0, m;
		for (int i = 0; i < n2 - 1; i++) {
			uint16_t count = nodes[i].count;
			if (r >= n) {
				r = 0;
				useX = !useX;
			}
			l = r;
			r += count;
			m = (l + r - 1) >> 1;
			int p = (i << 1) + 1;
			std::nth_element(ptr + l, ptr + m, ptr + r, [&useX](vec2 a, vec2 b) { return useX ? a.x < b.x : a.y < b.y; });
			minmax(ptr + l, ptr + m + 1, nodes[p]);
			minmax(ptr + m + 1, ptr + r, nodes[p + 1]);
			nodes[p].count = (count + 1) >> 1;
			nodes[p + 1].count = count >> 1;
		}
	}

	inline void minmax(vec2* first, vec2* last, node& node) {
		float min_x = first->x, max_x = first->x;
		float min_y = first->y, max_y = first->y;
		for (vec2* p = first + 1; p < last; p++) {
			float x = p->x, y = p->y;
			if (x < min_x)
				min_x = x;
			else if (x > max_x)
				max_x = x;
			if (y < min_y)
				min_y = y;
			else if (y > max_y)
				max_y = y;
		}
		node = { min_x, min_y, max_x, max_y };
	}

	node operator[](int i) {
		return nodes[i];
	}
};

void Task::checkVisible(const std::vector<unit>& input_units, std::vector<int>& result)
{
	int n = input_units.size();
	result.resize(n);

	std::vector<vec2> points(n);

	for (int i = 0; i < n; i++)
		points[i] = input_units[i].position;

	tree qtree(points);

	int threadsCount = std::max((int)std::thread::hardware_concurrency() - 1, 0);
	std::vector<std::thread> threads;
	threads.reserve(threadsCount);

	auto func = [&](int thread) {
		for (int i = thread; i < n; i += threadsCount + 1) {
			const unit* u = &input_units[i];
			float X = u->position.x, Y = u->position.y;
			float dist = u->distance;
			float dx = u->direction.x * dist, dy = u->direction.y * dist;

			float angle = u->fov_deg * 0.00872665f;
			float sina = sin(angle);
			float cosa = cos(angle);

			float term1 = dx * cosa;
			float term2 = dy * sina;
			float a11 = term1 + term2;
			float a21 = term1 - term2;

			term1 = dy * cosa;
			term2 = dx * sina;
			float a12 = term1 - term2;
			float a22 = term1 + term2;

			int type = 0;
			if (a21 > 0) type += 1;
			if (a22 > 0) type += 2;
			if (a11 > 0) type += 4;
			if (a12 > 0) type += 8;

			float min_x = X;
			float max_x = X;
			float min_y = Y;
			float max_y = Y;

			switch (type)
			{
			case(0):
				min_x += a11;
				min_y += a22;
				break;
			case(10):
				min_x += a21;
				max_y += a12;
				break;
			case(15):
				max_x += a11;
				max_y += a22;
				break;
			case(5):
				max_x += a21;
				min_y += a12;
				break;
			case(8):
				min_x -= dist;
				min_y += a22;
				max_y += a12;
				break;
			case(14):
				max_y += dist;
				min_x += a21;
				max_x += a11;
				break;
			case(7):
				max_x += dist;
				min_y += a12;
				max_y += a22;
				break;
			case(1):
				min_y -= dist;
				min_x += a11;
				max_x += a21;
				break;
			case(12):
				min_x -= dist;
				max_y += dist;
				max_x += a11;
				min_y += a22;
				break;
			case(6):
				max_x += dist;
				max_y += dist;
				min_x += a21;
				min_y += a12;
				break;
			case(3):
				max_x += dist;
				min_y -= dist;
				min_x += a11;
				max_y += a22;
				break;
			case(9):
				min_x -= dist;
				min_y -= dist;
				max_x += a21;
				max_y += a12;
			}

			float a1 = a12 / a11;
			float a2 = a22 / a21;
			float b1 = Y - a1 * X;
			float b2 = Y - a2 * X;
			dist *= dist;

			int stack[32];
			stack[0] = 0;
			int queue = 1;
			int res = 0;
			int n = qtree.nodes.size();

			if ((a11 < 0 ? Y <= a1 * X + b1 : Y >= a1 * X + b1) &&
				(a21 < 0 ? Y >= a2 * X + b2 : Y <= a2 * X + b2))
				res = -1;

			auto Check = [&](int i) {

				float min_x2 = qtree[i].min_x;
				if (min_x2 > max_x)
					return true;

				float max_x2 = qtree[i].max_x;
				if (max_x2 < min_x)
					return true;

				float min_y2 = qtree[i].min_y;
				if (min_y2 > max_y)
					return true;

				float max_y2 = qtree[i].max_y;
				if (max_y2 < min_y)
					return true;

				switch (type) {
				case (0):
					if (max_y2 > a1 * min_x2 + b1)
						return (min_y2 > a1 * max_x2 + b1 || max_y2 < a2 * min_x2 + b2);
					if (min_y2 < a2 * max_x2 + b2)
						return max_y2 < a2 * min_x2 + b2;
					if ((X - min_x2) * (X - min_x2) + (Y - min_y2) * (Y - min_y2) > dist)
						return abs(X - max_x2) * (X - max_x2) + abs(Y - max_y2) * (Y - max_y2) > dist;
					res += qtree[i].count;
					return true;

				case (1):
					if (max_y2 > a1 * min_x2 + b1)
						return (min_y2 > a1 * max_x2 + b1 || min_y2 > a2 * min_x2 + b2);
					if (max_y2 > a2 * max_x2 + b2)
						return min_y2 > a2 * min_x2 + b2;
					if ((X - max_x2) * (X - max_x2) + (Y - min_y2) * (Y - min_y2) > dist ||
						(X - min_x2) * (X - min_x2) + (Y - min_y2) * (Y - min_y2) > dist)
						return false;
					res += qtree[i].count;
					return true;

				case (3):
					if (max_y2 > a1 * min_x2 + b1)
						return (min_y2 > a1 * max_x2 + b1 || min_y2 > a2 * max_x2 + b2);
					if (max_y2 > a2 * min_x2 + b2)
						return min_y2 > a2 * max_x2 + b2;
					if ((X - max_x2) * (X - max_x2) + (Y - min_y2) * (Y - min_y2) > dist)
						return abs(Y - max_y2) * (Y - max_y2) - abs(X - min_x2) * (X - min_x2) > dist;
					if ((X - min_x2) * (X - min_x2) + (Y - min_y2) * (Y - min_y2) > dist ||
						(X - max_x2) * (X - max_x2) + (Y - max_y2) * (Y - max_y2) > dist)
						return false;
					res += qtree[i].count;
					return true;

				case (5):
					if (min_y2 < a1 * min_x2 + b1)
						return (max_y2 < a1* max_x2 + b1 || min_y2 > a2 * min_x2 + b2);
					if (max_y2 > a2 * max_x2 + b2)
						return min_y2 > a2 * min_x2 + b2;
					if ((X - max_x2) * (X - max_x2) + (Y - min_y2) * (Y - min_y2) > dist)
						return abs(Y - max_y2) * (Y - max_y2) - abs(X - min_x2) * (X - min_x2) > dist;
					res += qtree[i].count;
					return true;

				case (6):
					if (min_y2 < a1 * min_x2 + b1)
						return (max_y2 < a1 * max_x2 + b1 || max_y2 < a2* max_x2 + b2);
					if (min_y2 < a2 * min_x2 + b2)
						return max_y2 < a2 * max_x2 + b2;
					if ((X - max_x2) * (X - max_x2) + (Y - max_y2) * (Y - max_y2) > dist)
						return abs(X - min_x2) * (X - min_x2) + abs(Y - min_y2) * (Y - min_y2) < -dist;
					if ((X - max_x2) * (X - max_x2) + (Y - min_y2) * (Y - min_y2) > dist ||
						(X - min_x2) * (X - min_x2) + (Y - max_y2) * (Y - max_y2) > dist)
						return false;
					res += qtree[i].count;
					return true;

				case (7):
					if (min_y2 < a1 * min_x2 + b1)
						return (max_y2 < a1 * max_x2 + b1 || min_y2 > a2 * max_x2 + b2);
					if (max_y2 > a2 * min_x2 + b2)
						return min_y2 > a2 * max_x2 + b2;
					if ((X - max_x2) * (X - max_x2) + (Y - min_y2) * (Y - min_y2) > dist ||
						(X - max_x2) * (X - max_x2) + (Y - max_y2) * (Y - max_y2) > dist)
						return false;
					res += qtree[i].count;
					return true;

				case (8):
					if (max_y2 > a1 * max_x2 + b1)
						return (min_y2 > a1 * min_x2 + b1 || max_y2 < a2 * min_x2 + b2);
					if (min_y2 < a2 * max_x2 + b2)
						return max_y2 < a2* min_x2 + b2;
					if ((X - min_x2) * (X - min_x2) + (Y - min_y2) * (Y - min_y2) > dist ||
						(X - min_x2) * (X - min_x2) + (Y - max_y2) * (Y - max_y2) > dist)
						return false;
					res += qtree[i].count;
					return true;

				case (9):
					if (max_y2 > a1 * max_x2 + b1)
						return (min_y2 > a1 * min_x2 + b1 || min_y2 > a2 * min_x2 + b2);
					if (max_y2 > a2 * max_x2 + b2)
						return min_y2 > a2 * min_x2 + b2;
					if ((X - min_x2) * (X - min_x2) + (Y - min_y2) * (Y - min_y2) > dist)
						return abs(X - max_x2) * (X - max_x2) + abs(Y - max_y2) * (Y - max_y2) > dist;
					if ((X - max_x2) * (X - max_x2) + (Y - min_y2) * (Y - min_y2) > dist ||
						(X - min_x2) * (X - min_x2) + (Y - max_y2) * (Y - max_y2) > dist)
						return false;
					res += qtree[i].count;
					return true;

				case (10):
					if (max_y2 > a1 * max_x2 + b1)
						return (min_y2 > a1 * min_x2 + b1 || max_y2 < a2* max_x2 + b2);
					if (min_y2 < a2 * min_x2 + b2)
						return max_y2 < a2 * max_x2 + b2;
					if ((X - min_x2) * (X - min_x2) + (Y - max_y2) * (Y - max_y2) > dist)
						return abs(X - max_x2) * (X - max_x2) - abs(Y - min_y2) * (Y - min_y2) > dist;
					res += qtree[i].count;
					return true;

				case (12):
					if (min_y2 < a1 * max_x2 + b1)
						return (max_y2 < a1 * min_x2 + b1 || max_y2 < a2* min_x2 + b2);
					if (min_y2 < a2 * max_x2 + b2)
						return max_y2 < a2 * min_x2 + b2;
					if ((X - min_x2) * (X - min_x2) + (Y - max_y2) * (Y - max_y2) > dist)
						return abs(X - max_x2) * (X - max_x2) - abs(Y - min_y2) * (Y - min_y2) > dist;
					if ((X - max_x2) * (X - max_x2) + (Y - max_y2) * (Y - max_y2) > dist ||
						(X - min_x2) * (X - min_x2) + (Y - min_y2) * (Y - min_y2) > dist)
						return false;
					res += qtree[i].count;
					return true;

				case (14):
					if (min_y2 < a1 * max_x2 + b1)
						return (max_y2 < a1 * min_x2 + b1 || max_y2 < a2 * max_x2 + b2);
					if (min_y2 < a2 * min_x2 + b2)
						return max_y2 < a2* max_x2 + b2;
					if ((X - min_x2) * (X - min_x2) + (Y - max_y2) * (Y - max_y2) > dist ||
						(X - max_x2) * (X - max_x2) + (Y - max_y2) * (Y - max_y2) > dist)
						return false;
					res += qtree[i].count;
					return true;

				case (15):
					if (min_y2 < a1 * max_x2 + b1)
						return (max_y2 < a1* min_x2 + b1 || min_y2 > a2 * max_x2 + b2);
					if (max_y2 > a2 * min_x2 + b2)
						return min_y2 > a2 * max_x2 + b2;
					if ((X - max_x2) * (X - max_x2) + (Y - max_y2) * (Y - max_y2) > dist)
						return abs(X - min_x2) * (X - min_x2) + abs(Y - min_y2) * (Y - min_y2) < -dist;
					res += qtree[i].count;
					return true;
				}
			};

			while (queue)
			{
				int i = stack[--queue];

				while (true)
				{
					i = (i << 1) + 1;
					if (i >= n)
						break;

					if (Check(i))
					{
						if (Check(i + 1))
							break;
						else
							i++;
					}
					else
					{
						if (!Check(i + 1))
							stack[queue++] = i + 1;
					}
				}
			}
			result[i] = res;
		}
	};

	for (int i = 0; i < threadsCount; i++)
		threads.emplace_back(func, i);
	func(threadsCount);
	for (int i = 0; i < threadsCount; i++)
		threads[i].join();
}

