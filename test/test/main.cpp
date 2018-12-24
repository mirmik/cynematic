#include <linalg.h>
#include <linalg-ext.h>
#include <linalg-add.h>

#include <iostream>
#include <vector>

#include <cynematic/link.h>

using namespace linalg;
using namespace linalg::aliases;
using namespace linalg::ostream_overloads;

float deg(float angle)
{
	return angle * M_PI / 180.0;
}

int main ()
{
	linalg::mtrans<float> trans1(rotation_quat<float>(float3(0,0,1), deg(90)), float3(0,0,0));
	linalg::mtrans<float> trans2(rotation_quat<float>(float3(0,0,1), deg(0)), float3(0,10,0));

	linalg::bivec<float, 3> bv(float3(0,1,0), float3(0,1,0));

	nos::println(((trans2 * trans1)(bv)).concat());
	nos::println(trans2(trans1(bv)).concat());

	nos::println(((trans1 * trans2)(bv)).concat());
	nos::println(trans1(trans2(bv)).concat());


	/*vec<float,6> target {0,0,1,0,28,2};
	auto unit_target = linalg::normalize(target);
	
	vec<float,6> v0 {0,0,1,0,1,1};
	v0 = linalg::normalize(v0);
	
	vec<float,6> v1 {0,0,0,0,1,1};
	v1 = linalg::normalize(v1);

	vec<float,6> v2 {0,0,0,0,1,-1};
	v2 = linalg::normalize(v2);
	
	float kv0 = 0;
	float kv1 = 0;
	float kv2 = 0;

	auto flow = [&]() { return kv0 * v0 + kv1 * v1 + kv2 * v2; };
	auto print = [&]() { nos::println(unit_target - flow(), kv0, kv1, kv2); };

	auto correct0 = [&]() { return linalg::dot(unit_target - flow(), v0); };
	auto correct1 = [&]() { return linalg::dot(unit_target - flow(), v1); };
	auto correct2 = [&]() { return linalg::dot(unit_target - flow(), v2); };

	print();

	auto last = unit_target;
	do {
		last = unit_target - flow();
		
		auto k0 = correct0();
		kv0 += k0;

		auto k1 = correct1(); 
		kv1 += k1;

		auto k2 = correct2();
		kv2 += k2;

		print();
	} while(linalg::length(unit_target - flow() - last) > 0.000001);*/
} 