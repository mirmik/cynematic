#include <linalg.h>
#include <linalg-ext.h>
#include <linalg-add.h>

#include <iostream>
#include <vector>

#include <cynematic/link.h>
#include <cynematic/linalg-addon.h>

using namespace linalg;
using namespace linalg::aliases;
using namespace linalg::ostream_overloads;

using namespace cynematic;

float deg(float angle)
{
	return angle * M_PI / 180.0;
}

template<typename T, int M>
std::vector<T> backpack(vec<T,M> need, std::vector<vec<T,M>> sens) 
{
	vec<T,M> last_res;
	auto res = vec<T,M>{0,0,0};
	auto koeffs = std::vector<T>();
	koeffs.resize(sens.size());

	nos::println();
	do 
		for (int i = 0; i < sens.size(); ++i) 
		{
			last_res = res;
			PRINT(need);
			PRINT(res);
			PRINT(need - res);
			PRINT(sens[i]);

			PRINT(linalg::dot(need - res, sens[i]));

			koeffs[i] += linalg::dot(need - res, sens[i]) / length2(sens[i]);

			res = vec<T,M>{0,0,0};
			for(int j = 0; j < sens.size(); ++j) 
			{
				res += sens[j] * koeffs[j];
			}
		}
	while (length(res - last_res) > 0.000001);

	return koeffs;
}

int main ()
{
	cynematic::rotation_link<float> r0(float3(0,0,1));
	cynematic::constant_link<float> c0(mtrans<float>::translation(float3(0,0,5)));
	cynematic::translation_link<float> t0(float3(1,0,0));
	cynematic::translation_link<float> t1(float3(0,0,1));

	cynematic::chain<float> chain { &c0, &r0, &t0, &t1 };

	auto res = chain.get_speed_transes({deg(45),10,10});
	std::vector<float3> sens;
	std::vector<vec<float,6>> sens6;

	for (auto& r : res) {
		sens.push_back(r.b);
		sens6.push_back(r.concat());
	}

	float3 target {0,10,20};
	mtrans<float> target6 ({0,0,0,1},{0,0,0});

	float3 current = chain.get({0,10,10}).mov;
	mtrans<float> current6 = chain.get({0,10,10});

	auto need_to_target = target - current;
	auto need_to_target6 = target6 * current6.inverse();

	auto backp = backpack(normalize(need_to_target), sens);
	auto backp6 = backpack(normalize(need_to_target6.vector6()), sens6);

	PRINT(need_to_target);
	vec<float,3> accum;
	for (int i = 0; i < backp.size(); ++i) {
		accum += sens[i] * backp[i];
		PRINT(sens[i]);
		PRINT(backp[i]);
	}

	PRINT(need_to_target6.vector6());
	vec<float,6> accum6;
	for (int i = 0; i < backp6.size(); ++i) {
		accum6 += sens6[i] * backp6[i];
		PRINT(sens6[i]);
		PRINT(backp6[i]);
	}

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