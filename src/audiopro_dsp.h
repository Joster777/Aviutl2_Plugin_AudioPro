//----------------------------------------------------------------------------------
//	AudioPro DSP core  (Windows / AviUtl SDK に依存しない純粋なC++のみ)
//
//	- Equalizer        : ローカット/ハイカット(24dB/oct) + ローシェルフ + ピーキング3つ + ハイシェルフ
//	- Compressor       : ステレオリンク・ソフトニー・オートメイクアップ・パラレルMIX
//	- Limiter          : 先読みなしの簡易リミッター (ノーマライズ後のクリップ防止用)
//	- LoudnessMeter    : ITU-R BS.1770-4 のゲート付き統合ラウドネス (LUFS)
//	- LoudnessFollower : リアルタイムに目標ラウドネスへ追従するAGC
//	- NoiseReducer     : 遅延ゼロのマルチバンド適応エキスパンダ (定常ノイズ向け) + ハムカット
//	- Ducker           : ゲートのON/OFFからBGMの音量を下げるゲインスムーザ
//	- PitchShifter     : 速度を変えずに高さを変える (2タップ・クロスフェード方式 / 固定遅延 約window/2)
//
//	すべて「ストリーム処理」です。連続したチャンクを順番に渡してください。
//	(不連続になった時は reset() を呼びます)
//----------------------------------------------------------------------------------
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace apdsp
{
	constexpr double PI = 3.14159265358979323846;

	inline double db2lin(double db) { return std::pow(10.0, db / 20.0); }
	inline double lin2db(double x) { return 20.0 * std::log10(std::max(x, 1e-12)); }

	// 時定数(ms)から1サンプルあたりの1次ローパス係数を求める (0以下なら即応答)
	inline double smooth_coef(double ms, double sr)
	{
		if (ms <= 0.0)
		{
			return 1.0;
		}
		return 1.0 - std::exp(-1.0 / (ms * 0.001 * sr));
	}

	inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

	// NaN/Infや異常値を無音にする (他のフィルタから来た不正値で状態が壊れないように)
	inline double sanitize(double x)
	{
		if (!(x == x) || x > 1e6 || x < -1e6)
		{
			return 0.0;
		}
		return x;
	}

	//-----------------------------------------------------------------------------
	//	Biquad (RBJ Audio EQ Cookbook)
	//-----------------------------------------------------------------------------
	struct BiquadCoef
	{
		double b0 = 1.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0;
	};

	enum class FilterType { LowPass, HighPass, LowShelf, HighShelf, Peak, Notch };

	inline BiquadCoef design_biquad(FilterType type, double sr, double freq, double q, double gain_db)
	{
		freq = clampd(freq, 5.0, sr * 0.49);
		q = std::max(q, 0.05);
		const double w0 = 2.0 * PI * freq / sr;
		const double cs = std::cos(w0);
		const double sn = std::sin(w0);
		const double alpha = sn / (2.0 * q);
		const double A = std::pow(10.0, gain_db / 40.0);
		const double sqA2alpha = 2.0 * std::sqrt(A) * alpha;
		double b0 = 1, b1 = 0, b2 = 0, a0 = 1, a1 = 0, a2 = 0;
		switch (type)
		{
		case FilterType::LowPass:
			b0 = (1 - cs) / 2; b1 = 1 - cs; b2 = (1 - cs) / 2;
			a0 = 1 + alpha; a1 = -2 * cs; a2 = 1 - alpha;
			break;
		case FilterType::HighPass:
			b0 = (1 + cs) / 2; b1 = -(1 + cs); b2 = (1 + cs) / 2;
			a0 = 1 + alpha; a1 = -2 * cs; a2 = 1 - alpha;
			break;
		case FilterType::Peak:
			b0 = 1 + alpha * A; b1 = -2 * cs; b2 = 1 - alpha * A;
			a0 = 1 + alpha / A; a1 = -2 * cs; a2 = 1 - alpha / A;
			break;
		case FilterType::Notch:
			b0 = 1; b1 = -2 * cs; b2 = 1;
			a0 = 1 + alpha; a1 = -2 * cs; a2 = 1 - alpha;
			break;
		case FilterType::LowShelf:
			b0 = A * ((A + 1) - (A - 1) * cs + sqA2alpha);
			b1 = 2 * A * ((A - 1) - (A + 1) * cs);
			b2 = A * ((A + 1) - (A - 1) * cs - sqA2alpha);
			a0 = (A + 1) + (A - 1) * cs + sqA2alpha;
			a1 = -2 * ((A - 1) + (A + 1) * cs);
			a2 = (A + 1) + (A - 1) * cs - sqA2alpha;
			break;
		case FilterType::HighShelf:
			b0 = A * ((A + 1) + (A - 1) * cs + sqA2alpha);
			b1 = -2 * A * ((A - 1) + (A + 1) * cs);
			b2 = A * ((A + 1) + (A - 1) * cs - sqA2alpha);
			a0 = (A + 1) - (A - 1) * cs + sqA2alpha;
			a1 = 2 * ((A - 1) - (A + 1) * cs);
			a2 = (A + 1) - (A - 1) * cs - sqA2alpha;
			break;
		}
		BiquadCoef c;
		c.b0 = b0 / a0; c.b1 = b1 / a0; c.b2 = b2 / a0;
		c.a1 = a1 / a0; c.a2 = a2 / a0;
		return c;
	}

	// 周波数応答 (dB) テスト・確認用
	inline double biquad_mag_db(const BiquadCoef& c, double freq, double sr)
	{
		const double w = 2.0 * PI * freq / sr;
		const double cw = std::cos(w), sw = std::sin(w);
		const double c2 = std::cos(2 * w), s2 = std::sin(2 * w);
		const double nr = c.b0 + c.b1 * cw + c.b2 * c2;
		const double ni = -(c.b1 * sw + c.b2 * s2);
		const double dr = 1 + c.a1 * cw + c.a2 * c2;
		const double di = -(c.a1 * sw + c.a2 * s2);
		return 10.0 * std::log10((nr * nr + ni * ni) / (dr * dr + di * di));
	}

	// 転置直接形II。係数を途中で変えても破綻しにくい
	struct Biquad
	{
		BiquadCoef c;
		double z1 = 0.0, z2 = 0.0;
		inline double process(double x)
		{
			double y = c.b0 * x + z1;
			z1 = c.b1 * x - c.a1 * y + z2;
			z2 = c.b2 * x - c.a2 * y;
			// 非正規化数対策
			if (std::fabs(z1) < 1e-30) z1 = 0.0;
			if (std::fabs(z2) < 1e-30) z2 = 0.0;
			return y;
		}
		void reset() { z1 = z2 = 0.0; }
	};

	//-----------------------------------------------------------------------------
	//	Equalizer
	//-----------------------------------------------------------------------------
	struct EqParams
	{
		double hp_freq = 0.0;		// ローカット (Hz, 0=オフ)  24dB/oct
		double lp_freq = 0.0;		// ハイカット (Hz, 0=オフ)  24dB/oct
		// [0]=ローシェルフ [1][2][3]=ピーキング [4]=ハイシェルフ
		double freq[5] = { 100.0, 250.0, 1000.0, 3500.0, 8000.0 };
		double gain[5] = { 0, 0, 0, 0, 0 };		// dB
		double q[5] = { 0.7071, 1.0, 1.0, 1.0, 0.7071 };
	};

	struct Equalizer
	{
		Biquad hp[2][2];		// [stage][channel]
		Biquad lp[2][2];
		Biquad band[5][2];		// [band][channel]
		bool hp_on = false, lp_on = false;
		bool band_on[5] = {};

		void reset()
		{
			for (auto& s : hp) for (auto& b : s) b.reset();
			for (auto& s : lp) for (auto& b : s) b.reset();
			for (auto& s : band) for (auto& b : s) b.reset();
		}

		void configure(const EqParams& p, double sr)
		{
			// 4次バターワース = Q 0.5412 と 1.3066 の2段
			static const double butter4_q[2] = { 0.54119610, 1.30656296 };
			hp_on = p.hp_freq > 1.0;
			lp_on = p.lp_freq > 1.0 && p.lp_freq < sr * 0.49;
			for (int s = 0; s < 2; s++)
			{
				if (hp_on)
				{
					BiquadCoef c = design_biquad(FilterType::HighPass, sr, p.hp_freq, butter4_q[s], 0);
					hp[s][0].c = c; hp[s][1].c = c;
				}
				else
				{
					hp[s][0].reset(); hp[s][1].reset();
				}
				if (lp_on)
				{
					BiquadCoef c = design_biquad(FilterType::LowPass, sr, p.lp_freq, butter4_q[s], 0);
					lp[s][0].c = c; lp[s][1].c = c;
				}
				else
				{
					lp[s][0].reset(); lp[s][1].reset();
				}
			}
			static const FilterType types[5] = { FilterType::LowShelf, FilterType::Peak, FilterType::Peak, FilterType::Peak, FilterType::HighShelf };
			for (int i = 0; i < 5; i++)
			{
				band_on[i] = std::fabs(p.gain[i]) > 0.005 && p.freq[i] < sr * 0.49;
				if (band_on[i])
				{
					BiquadCoef c = design_biquad(types[i], sr, p.freq[i], p.q[i], p.gain[i]);
					band[i][0].c = c; band[i][1].c = c;
				}
				else
				{
					band[i][0].reset(); band[i][1].reset();
				}
			}
		}

		void process(float* ch0, float* ch1, int n)
		{
			float* data[2] = { ch0, ch1 };
			for (int ch = 0; ch < 2; ch++)
			{
				float* x = data[ch];
				if (!x)
				{
					continue;
				}
				for (int i = 0; i < n; i++)
				{
					double v = sanitize(x[i]);
					if (hp_on) { v = hp[0][ch].process(v); v = hp[1][ch].process(v); }
					for (int b = 0; b < 5; b++)
					{
						if (band_on[b]) v = band[b][ch].process(v);
					}
					if (lp_on) { v = lp[0][ch].process(v); v = lp[1][ch].process(v); }
					x[i] = (float)v;
				}
			}
		}
	};

	//-----------------------------------------------------------------------------
	//	Compressor
	//-----------------------------------------------------------------------------
	struct CompressorParams
	{
		double threshold_db = -18.0;
		double ratio = 3.0;
		double attack_ms = 10.0;
		double release_ms = 120.0;
		double knee_db = 6.0;
		double makeup_db = 0.0;
		bool auto_makeup = false;	// 0dBFSの入力が0dBFSで出る量を自動で足す
		double mix = 1.0;			// 1.0=全部圧縮後 / 0.0=原音 (パラレルコンプ)
	};

	// 圧縮カーブ: 入力レベル(dB)に対するゲイン変化(dB, 0以下)
	inline double compressor_gain_db(double level_db, const CompressorParams& p)
	{
		const double T = p.threshold_db;
		const double R = std::max(p.ratio, 1.0);
		const double W = std::max(p.knee_db, 0.0);
		const double over = level_db - T;
		double out;
		if (2.0 * over < -W)
		{
			out = level_db;
		}
		else if (2.0 * std::fabs(over) <= W)
		{
			const double t = over + W / 2.0;
			out = level_db + (1.0 / R - 1.0) * t * t / (2.0 * W);
		}
		else
		{
			out = T + over / R;
		}
		return out - level_db;
	}

	struct Compressor
	{
		double gr_db = 0.0;		// 現在のゲインリダクション(dB, 0以下)
		void reset() { gr_db = 0.0; }

		void process(float* ch0, float* ch1, int n, double sr, const CompressorParams& p)
		{
			const double atk = smooth_coef(p.attack_ms, sr);
			const double rel = smooth_coef(p.release_ms, sr);
			double makeup = p.makeup_db;
			if (p.auto_makeup)
			{
				makeup += -p.threshold_db * (1.0 - 1.0 / std::max(p.ratio, 1.0));
				if (makeup < p.makeup_db) makeup = p.makeup_db;
			}
			const double mix = clampd(p.mix, 0.0, 1.0);
			const double makeup_lin = db2lin(makeup);
			for (int i = 0; i < n; i++)
			{
				const double l = sanitize(ch0[i]);
				const double r = ch1 ? sanitize(ch1[i]) : l;
				const double level = std::max(std::fabs(l), std::fabs(r));
				const double target = compressor_gain_db(lin2db(level), p);
				// ゲインリダクションが増える時がアタック、戻る時がリリース
				const double coef = (target < gr_db) ? atk : rel;
				gr_db += (target - gr_db) * coef;
				const double g = db2lin(gr_db) * makeup_lin;
				const double gm = (1.0 - mix) + mix * g;
				ch0[i] = (float)(l * gm);
				if (ch1) ch1[i] = (float)(r * gm);
			}
		}
	};

	//-----------------------------------------------------------------------------
	//	Limiter (先読みなし。アタック即時、リリースのみ時定数)
	//-----------------------------------------------------------------------------
	struct Limiter
	{
		double gain = 1.0;
		void reset() { gain = 1.0; }
		void process(float* ch0, float* ch1, int n, double sr, double ceiling_db, double release_ms)
		{
			const double ceil_lin = db2lin(ceiling_db);
			const double rel = smooth_coef(release_ms, sr);
			for (int i = 0; i < n; i++)
			{
				const double l = sanitize(ch0[i]);
				const double r = ch1 ? sanitize(ch1[i]) : l;
				const double peak = std::max(std::fabs(l), std::fabs(r));
				const double target = (peak > ceil_lin) ? ceil_lin / peak : 1.0;
				if (target < gain)
				{
					gain = target;
				}
				else
				{
					gain += (1.0 - gain) * rel;
					if (gain > target) gain = target;
				}
				ch0[i] = (float)(l * gain);
				if (ch1) ch1[i] = (float)(r * gain);
			}
		}
	};

	//-----------------------------------------------------------------------------
	//	ラウドネス (ITU-R BS.1770-4 / EBU R128)
	//-----------------------------------------------------------------------------
	inline void kweight_coefs(double sr, BiquadCoef& shelf, BiquadCoef& hp)
	{
		{
			const double G = 3.999843853973347, Q = 0.7071752369554196, fc = 1681.974450955533;
			const double K = std::tan(PI * fc / sr);
			const double Vh = std::pow(10.0, G / 20.0);
			const double Vb = std::pow(Vh, 0.4996667741545416);
			const double a0 = 1.0 + K / Q + K * K;
			shelf.b0 = (Vh + Vb * K / Q + K * K) / a0;
			shelf.b1 = 2.0 * (K * K - Vh) / a0;
			shelf.b2 = (Vh - Vb * K / Q + K * K) / a0;
			shelf.a1 = 2.0 * (K * K - 1.0) / a0;
			shelf.a2 = (1.0 - K / Q + K * K) / a0;
		}
		{
			const double Q = 0.5003270373238773, fc = 38.13547087602444;
			const double K = std::tan(PI * fc / sr);
			const double a0 = 1.0 + K / Q + K * K;
			hp.b0 = 1.0; hp.b1 = -2.0; hp.b2 = 1.0;
			hp.a1 = 2.0 * (K * K - 1.0) / a0;
			hp.a2 = (1.0 - K / Q + K * K) / a0;
		}
	}

	struct KWeight
	{
		Biquad shelf, hp;
		void init(double sr) { kweight_coefs(sr, shelf.c, hp.c); shelf.reset(); hp.reset(); }
		inline double process(double x) { return hp.process(shelf.process(x)); }
	};

	inline double power_to_lufs(double power)
	{
		return -0.691 + 10.0 * std::log10(std::max(power, 1e-20));
	}

	// ファイル全体など、まとめて解析するためのメーター
	struct LoudnessMeter
	{
		KWeight kw[2];
		int sub_len = 4800;			// 100ms
		double sub_sum = 0.0;
		int sub_cnt = 0;
		std::vector<double> sub_powers;
		std::vector<double> block_powers;	// 400ms(75%オーバーラップ)
		double peak = 0.0;

		void init(double sr)
		{
			kw[0].init(sr); kw[1].init(sr);
			sub_len = std::max(1, (int)std::lround(sr * 0.1));
			sub_sum = 0.0; sub_cnt = 0;
			sub_powers.clear(); block_powers.clear();
			peak = 0.0;
		}

		void push(const float* l, const float* r, int n)
		{
			for (int i = 0; i < n; i++)
			{
				const double xl = sanitize(l[i]);
				const double xr = r ? sanitize(r[i]) : xl;
				peak = std::max(peak, std::max(std::fabs(xl), std::fabs(xr)));
				const double yl = kw[0].process(xl);
				const double yr = kw[1].process(xr);
				sub_sum += yl * yl + yr * yr;
				if (++sub_cnt >= sub_len)
				{
					sub_powers.push_back(sub_sum / sub_len);
					sub_sum = 0.0; sub_cnt = 0;
					const size_t m = sub_powers.size();
					if (m >= 4)
					{
						block_powers.push_back((sub_powers[m - 1] + sub_powers[m - 2] + sub_powers[m - 3] + sub_powers[m - 4]) / 4.0);
					}
				}
			}
		}

		// ゲート付き統合ラウドネス。測れない(無音など)場合は false
		bool integrated(double* lufs) const
		{
			std::vector<double> blocks = block_powers;
			if (blocks.empty())
			{
				// 400ms未満の短い素材は、100msブロックの平均で代用する
				if (sub_powers.empty() && sub_cnt > 0)
				{
					blocks.push_back(sub_sum / sub_cnt);
				}
				else
				{
					blocks = sub_powers;
				}
			}
			if (blocks.empty())
			{
				return false;
			}
			// 絶対ゲート -70 LUFS
			double sum = 0.0; size_t count = 0;
			for (double p : blocks)
			{
				if (power_to_lufs(p) > -70.0) { sum += p; count++; }
			}
			if (count == 0)
			{
				return false;
			}
			// 相対ゲート (絶対ゲート通過分の平均 - 10 LU)
			const double rel_threshold = power_to_lufs(sum / count) - 10.0;
			double sum2 = 0.0; size_t count2 = 0;
			for (double p : blocks)
			{
				const double l = power_to_lufs(p);
				if (l > -70.0 && l > rel_threshold) { sum2 += p; count2++; }
			}
			if (count2 == 0)
			{
				return false;
			}
			*lufs = power_to_lufs(sum2 / count2);
			return true;
		}
	};

	//-----------------------------------------------------------------------------
	//	リアルタイム・ラウドネス追従 (AGC)
	//-----------------------------------------------------------------------------
	struct FollowerParams
	{
		double target_lufs = -16.0;
		double window_s = 3.0;			// 測定の平均化時間
		double max_boost_db = 18.0;
		double max_cut_db = 18.0;
		double speed_db_per_s = 6.0;	// ゲイン変化の最大速度
		double gate_lufs = -55.0;		// これより小さいチャンク(無音)は測定・追従に使わない
	};

	struct LoudnessFollower
	{
		KWeight kw[2];
		double sr_ = 0.0;
		double power_avg = 0.0;
		double gated_samples = 0.0;
		double gain_db = 0.0;
		bool valid = false;

		void reset() { power_avg = 0.0; gated_samples = 0.0; gain_db = 0.0; valid = false; sr_ = 0.0; }

		double measured_lufs() const { return valid ? power_to_lufs(power_avg) : -200.0; }

		void process(float* ch0, float* ch1, int n, double sr, const FollowerParams& p)
		{
			if (n <= 0)
			{
				return;
			}
			if (sr_ != sr)
			{
				kw[0].init(sr); kw[1].init(sr);
				sr_ = sr;
			}
			// このチャンクの(入力側の)K特性パワー
			double sum = 0.0;
			for (int i = 0; i < n; i++)
			{
				const double xl = sanitize(ch0[i]);
				const double xr = ch1 ? sanitize(ch1[i]) : xl;
				const double yl = kw[0].process(xl);
				const double yr = kw[1].process(xr);
				sum += yl * yl + yr * yr;
			}
			const double chunk_power = sum / n;
			if (power_to_lufs(chunk_power) > p.gate_lufs)
			{
				gated_samples += n;
				if (!valid)
				{
					power_avg = chunk_power;
					valid = true;
				}
				else
				{
					// 最初は短い窓で素早く、貯まったら指定の窓で平均化する
					const double w = std::min(p.window_s * sr, std::max(gated_samples, 0.4 * sr));
					const double alpha = std::min(1.0, n / w);
					power_avg += (chunk_power - power_avg) * alpha;
				}
			}
			double desired = gain_db;
			if (valid)
			{
				desired = clampd(p.target_lufs - power_to_lufs(power_avg), -p.max_cut_db, p.max_boost_db);
			}
			const double max_step = p.speed_db_per_s * n / sr;
			const double new_gain_db = gain_db + clampd(desired - gain_db, -max_step, max_step);

			// チャンク内でゲインを滑らかに変える
			const double g0 = db2lin(gain_db), g1 = db2lin(new_gain_db);
			for (int i = 0; i < n; i++)
			{
				const double g = g0 + (g1 - g0) * ((i + 1.0) / n);
				ch0[i] = (float)(sanitize(ch0[i]) * g);
				if (ch1) ch1[i] = (float)(sanitize(ch1[i]) * g);
			}
			gain_db = new_gain_db;
		}
	};

	//-----------------------------------------------------------------------------
	//	ノイズ除去: 遅延ゼロのマルチバンド適応エキスパンダ
	//	帯域分割は LP_k(x) の差分なので、全ゲイン1のとき入力と完全に一致します
	//-----------------------------------------------------------------------------
	struct NoiseParams
	{
		double reduction_db = 12.0;		// 最大低減量
		double sensitivity_db = 5.0;	// ノイズ床の何dB上から音を通し始めるか (ここまではノイズ扱いで最大まで下げる)
		double knee_db = 6.0;			// 通し始めてから完全に開くまでの幅
		double attack_ms = 5.0;			// 音が来た時に開く速さ
		double release_ms = 120.0;		// 閉じる速さ
		double floor_rise_db_s = 1.0;	// ノイズ床の追従(上昇)速度
		int hum_hz = 0;					// 0=なし / 50 / 60
	};

	struct NoiseReducer
	{
		static constexpr int NB = 16;			// 帯域数
		static constexpr int NC = NB - 1;		// 分割点の数
		double sr_ = 0.0;
		Biquad lp[NC][2][2];					// [分割点][段][チャンネル]
		Biquad notch[3][2];
		double env[NB];
		double floor_[NB];
		double gain_db[NB];
		bool started = false;
		double cutoffs[NC];
		double prime_sum[NB];
		double env_coef_[NB];					// 帯域ごとの平均化の速さ (狭い帯域ほど長く平均する)
		long long prime_count = 0;

		NoiseReducer() { reset(); }

		void reset()
		{
			started = false;
			prime_count = 0;
			sr_ = 0.0;
			for (int k = 0; k < NB; k++) { env[k] = 0.0; floor_[k] = 1e-3; gain_db[k] = 0.0; prime_sum[k] = 0.0; }
		}

		void setup(double sr)
		{
			sr_ = sr;
			static const double q4[2] = { 0.54119610, 1.30656296 };
			for (int k = 0; k < NC; k++)
			{
				cutoffs[k] = 100.0 * std::pow(12000.0 / 100.0, (double)k / (NC - 1));
				for (int s = 0; s < 2; s++)
				{
					const BiquadCoef c = design_biquad(FilterType::LowPass, sr, cutoffs[k], q4[s], 0);
					lp[k][s][0].c = c; lp[k][s][1].c = c;
					lp[k][s][0].reset(); lp[k][s][1].reset();
				}
			}
			for (auto& b : notch) { b[0].reset(); b[1].reset(); }
			// 帯域幅が狭いとパワーの揺らぎが大きいので、帯域幅に反比例した時定数(5ms～60ms)で平均化する
			for (int k = 0; k < NB; k++)
			{
				const double lo = (k == 0) ? 0.0 : cutoffs[k - 1];
				const double hi = (k == NB - 1) ? sr * 0.5 : cutoffs[k];
				const double bw = std::max(hi - lo, 1.0);
				env_coef_[k] = smooth_coef(clampd(10.0 / bw, 0.005, 0.06) * 1000.0, sr);
			}
		}

		void process(float* ch0, float* ch1, int n, double sr, const NoiseParams& p)
		{
			if (sr_ != sr)
			{
				setup(sr);
			}
			const int channels = ch1 ? 2 : 1;
			// ハムカット (基本波と2,3倍音)
			if (p.hum_hz > 0)
			{
				for (int h = 0; h < 3; h++)
				{
					const BiquadCoef c = design_biquad(FilterType::Notch, sr, p.hum_hz * (h + 1.0), 25.0, 0);
					notch[h][0].c = c; notch[h][1].c = c;
				}
			}
			float* data[2] = { ch0, ch1 };

			const double atk = smooth_coef(p.attack_ms, sr);
			const double rel = smooth_coef(p.release_ms, sr);
			const double rise = std::pow(10.0, p.floor_rise_db_s / 10.0 / sr);	// パワー領域の1サンプルあたり上昇率
			const double max_red = std::max(p.reduction_db, 0.0);
			const double sens = p.sensitivity_db;
			const double knee = std::max(p.knee_db, 0.5);
			const long long prime_len = (long long)(0.1 * sr);	// 最初の100msでノイズ床の初期値を測る

			for (int i = 0; i < n; i++)
			{
				double band[2][NB];
				double in[2];
				for (int ch = 0; ch < channels; ch++)
				{
					double x = sanitize(data[ch][i]);
					if (p.hum_hz > 0)
					{
						for (int h = 0; h < 3; h++) x = notch[h][ch].process(x);
					}
					in[ch] = x;
					double prev = 0.0;
					for (int k = 0; k < NC; k++)
					{
						double y = lp[k][0][ch].process(x);
						y = lp[k][1][ch].process(y);
						band[ch][k] = y - prev;
						prev = y;
					}
					band[ch][NB - 1] = x - prev;
				}

				// 帯域ごとのエンベロープとノイズ床、目標ゲイン
				double target[NB];
				for (int k = 0; k < NB; k++)
				{
					double power = 0.0;
					for (int ch = 0; ch < channels; ch++) power += band[ch][k] * band[ch][k];
					power /= channels;
					env[k] += (power - env[k]) * env_coef_[k];
					if (!started)
					{
						// 測定中は何も処理せず、平均パワーをノイズ床の初期値にする
						prime_sum[k] += power;
						if (prime_count + 1 >= prime_len)
						{
							floor_[k] = std::max(prime_sum[k] / (double)prime_len, 1e-14);
							env[k] = floor_[k];
						}
						target[k] = 0.0;
						continue;
					}
					if (env[k] < floor_[k])
					{
						floor_[k] = std::max(env[k], 1e-14);
					}
					else
					{
						floor_[k] *= rise;
					}
					// ノイズ床からの高さ(dB)。sens未満は最大低減、sens+knee以上は素通し、間はなめらかに
					const double above = 10.0 * std::log10(std::max(env[k], 1e-20) / floor_[k]);
					double t = clampd((above - sens) / knee, 0.0, 1.0);
					t = t * t * (3.0 - 2.0 * t);
					target[k] = -max_red * (1.0 - t);
				}
				if (!started)
				{
					prime_count++;
					if (prime_count >= prime_len) started = true;
				}

				// 隣の帯域とゲインをならす (ミュージカルノイズ対策)
				double smoothed[NB];
				for (int k = 0; k < NB; k++)
				{
					const double a = target[k > 0 ? k - 1 : k];
					const double c = target[k < NB - 1 ? k + 1 : k];
					smoothed[k] = 0.25 * a + 0.5 * target[k] + 0.25 * c;
				}
				double gain[NB];
				for (int k = 0; k < NB; k++)
				{
					const double coef = (smoothed[k] > gain_db[k]) ? atk : rel;
					gain_db[k] += (smoothed[k] - gain_db[k]) * coef;
					gain[k] = db2lin(gain_db[k]);
				}

				for (int ch = 0; ch < channels; ch++)
				{
					double out = 0.0;
					for (int k = 0; k < NB; k++) out += band[ch][k] * gain[k];
					data[ch][i] = (float)out;
				}
			}
		}
	};

	//-----------------------------------------------------------------------------
	//	ダッキング用ゲインスムーザ
	//-----------------------------------------------------------------------------
	struct DuckParams
	{
		double depth_db = 12.0;		// 下げる量
		double attack_ms = 80.0;
		double hold_ms = 300.0;
		double release_ms = 500.0;
	};

	struct Ducker
	{
		double gain = 1.0;
		double hold_left = 0.0;
		void reset() { gain = 1.0; hold_left = 0.0; }

		// active: 1サンプルごとの「声がある」フラグ (nullptr なら声なし)
		void process(float* ch0, float* ch1, int n, double sr, const uint8_t* active, const DuckParams& p)
		{
			const double duck = db2lin(-std::fabs(p.depth_db));
			const double atk = smooth_coef(p.attack_ms, sr);
			const double rel = smooth_coef(p.release_ms, sr);
			const double hold = p.hold_ms * 0.001 * sr;
			for (int i = 0; i < n; i++)
			{
				const bool on = active && active[i];
				if (on)
				{
					hold_left = hold;
				}
				double target = 1.0;
				if (on || hold_left > 0.0)
				{
					target = duck;
					if (!on) hold_left -= 1.0;
				}
				gain += (target - gain) * (target < gain ? atk : rel);
				ch0[i] = (float)(sanitize(ch0[i]) * gain);
				if (ch1) ch1[i] = (float)(sanitize(ch1[i]) * gain);
			}
		}
	};

	//-----------------------------------------------------------------------------
	//	ピッチシフト: 2タップ・クロスフェード(遅延線の読み出し位置を連続的にずらす)
	//	再生時間は変えず、高さだけを変える。固定遅延は約 window/2
	//-----------------------------------------------------------------------------
	struct PitchParams
	{
		double semitones = 0.0;		// 半音単位 (+で高く、-で低く)
		double window_ms = 50.0;	// 窓の長さ
	};

	struct PitchShifter
	{
		std::vector<float> buf[2];
		int mask = 0;
		int wpos = 0;
		double phase = 0.0;			// 0..1 (タップAの窓位相。タップBは+0.5)
		double sr_ = 0.0;

		void reset()
		{
			for (auto& b : buf) std::fill(b.begin(), b.end(), 0.0f);
			wpos = 0;
			phase = 0.0;
		}

		void setup(double sr)
		{
			sr_ = sr;
			int size = 1;
			while (size < (int)(0.5 * sr)) size <<= 1;		// 窓の最大(200ms)より十分大きく
			mask = size - 1;
			for (auto& b : buf) b.assign((size_t)size, 0.0f);
			wpos = 0;
			phase = 0.0;
		}

		// 遅延線の cubic (Catmull-Rom) 補間読み出し。delay は wpos からの遅延(サンプル, 小数可)
		inline double tap(const std::vector<float>& b, double delay) const
		{
			const double rp = (double)wpos - delay;
			const double fl = std::floor(rp);
			const double f = rp - fl;
			const int i1 = (int)fl;
			const double y0 = b[(size_t)((i1 - 1) & mask)];
			const double y1 = b[(size_t)(i1 & mask)];
			const double y2 = b[(size_t)((i1 + 1) & mask)];
			const double y3 = b[(size_t)((i1 + 2) & mask)];
			const double c1 = 0.5 * (y2 - y0);
			const double c2 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
			const double c3 = 0.5 * (y3 - y0) + 1.5 * (y1 - y2);
			return ((c3 * f + c2) * f + c1) * f + y1;
		}

		void process(float* ch0, float* ch1, int n, double sr, const PitchParams& p)
		{
			if (sr_ != sr || buf[0].empty())
			{
				setup(sr);
			}
			const int channels = ch1 ? 2 : 1;
			float* data[2] = { ch0, ch1 };
			const double ratio = std::pow(2.0, clampd(p.semitones, -48.0, 48.0) / 12.0);
			const bool bypass = std::fabs(ratio - 1.0) < 1e-6;
			const double W = clampd(p.window_ms, 5.0, 200.0) * 0.001 * sr;
			const double dphase = (1.0 - ratio) / W;		// 1サンプルあたりの位相の進み
			const double base = 4.0;						// 補間に必要な余白(サンプル)

			for (int i = 0; i < n; i++)
			{
				double in[2] = { 0.0, 0.0 };
				for (int ch = 0; ch < channels; ch++)
				{
					in[ch] = sanitize(data[ch][i]);
					buf[ch][(size_t)(wpos & mask)] = (float)in[ch];
				}
				if (bypass)
				{
					// 素通し (遅延線は書き続けるので、切り替えても破綻しない)
					for (int ch = 0; ch < channels; ch++) data[ch][i] = (float)in[ch];
				}
				else
				{
					phase += dphase;
					phase -= std::floor(phase);
					double pb = phase + 0.5;
					if (pb >= 1.0) pb -= 1.0;
					const double sa = std::sin(PI * phase);
					const double sb = std::sin(PI * pb);
					const double wa = sa * sa;				// Hann窓。wa + wb = 1
					const double wb = sb * sb;
					const double da = base + phase * W;
					const double db = base + pb * W;
					for (int ch = 0; ch < channels; ch++)
					{
						const double y = tap(buf[ch], da) * wa + tap(buf[ch], db) * wb;
						data[ch][i] = (float)y;
					}
				}
				wpos = (wpos + 1) & mask;
			}
		}
	};
}