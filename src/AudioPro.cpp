//----------------------------------------------------------------------------------
//	AudioPro for AviUtl ExEdit2   version 0.1
//
//	音声の高度な処理をまとめた汎用プラグイン (.aux2)
//	フィルタ効果として次の6つを追加します (オブジェクトの「フィルタ効果の追加」から選べます)
//
//	  AudioPro ノイズ除去      : 定常ノイズ(エアコン・PCファン・ホワイトノイズ)を低減 / ハムノイズ(50/60Hz)のカット
//	  AudioPro イコライザー    : ローカット・ハイカット + 5バンド (シェルフ2 + ピーキング3) + プリセット
//	  AudioPro コンプレッサー  : しきい値・レシオ・アタック・リリース・ニー・メイクアップ・ミックス(パラレル)
//	  AudioPro ラウドネス調整  : 目標LUFSへ自動で音量を揃える (自動追従 / ファイル全体を解析して一定ゲイン)
//	  AudioPro ダッキング      : 指定レイヤーの声に合わせて、このオブジェクト(BGM)の音量を自動で下げる
//	  AudioPro ピッチシフト    : 速度を変えずに声の高さを変える (半音単位)
//
//	処理の本体は audiopro_dsp.h にあります (Windows非依存)
//----------------------------------------------------------------------------------
#define NOMINMAX
#include <windows.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cwctype>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "filter2.h"
#include "plugin2.h"
#include "cache2.h"
#include "config2.h"
#include "logger2.h"

#include "audiopro_dsp.h"

static const wchar_t* const PLUGIN_INFO = L"AudioPro version 0.1";

//---------------------------------------------------------------------
//	ホストから受け取るハンドル
//---------------------------------------------------------------------
static CACHE_HANDLE* g_cache = nullptr;
static CONFIG_HANDLE* g_config = nullptr;
static LOG_HANDLE* g_logger = nullptr;

static void log_text(LPCWSTR text)
{
	if (g_logger)
	{
		g_logger->log(g_logger, text);
	}
}

//---------------------------------------------------------------------
//	文字列ユーティリティ
//---------------------------------------------------------------------
static std::wstring utf8_to_wide(const char* text, size_t length)
{
	if (!text || length == 0)
	{
		return std::wstring();
	}
	int count = MultiByteToWideChar(CP_UTF8, 0, text, (int)length, nullptr, 0);
	if (count <= 0)
	{
		return std::wstring();
	}
	std::wstring result((size_t)count, L'\0');
	MultiByteToWideChar(CP_UTF8, 0, text, (int)length, &result[0], count);
	return result;
}

static std::string trim_ascii(const std::string& text)
{
	size_t start = text.find_first_not_of(" \t\r");
	if (start == std::string::npos)
	{
		return std::string();
	}
	size_t end = text.find_last_not_of(" \t\r");
	return text.substr(start, end - start + 1);
}

static std::wstring to_lower(std::wstring text)
{
	std::transform(text.begin(), text.end(), text.begin(), [](wchar_t c) { return (wchar_t)towlower(c); });
	return text;
}

//---------------------------------------------------------------------
//	設定項目の組み立て補助
//	(項目の構造体は本体から現在値を書き込まれるので、アドレスが変わらないよう deque に保持する)
//---------------------------------------------------------------------
struct EffectDef
{
	std::deque<FILTER_ITEM_TRACK> tracks;
	std::deque<FILTER_ITEM_CHECK> checks;
	std::deque<FILTER_ITEM_SELECT> selects;
	std::deque<std::vector<FILTER_ITEM_SELECT::ITEM>> lists;
	std::deque<FILTER_ITEM_GROUP> groups;
	std::deque<FILTER_ITEM_SEPARATOR> separators;
	std::deque<FILTER_ITEM_HIDE_RULE> hides;
	std::vector<void*> items;
	FILTER_PLUGIN_TABLE table = {};

	FILTER_ITEM_TRACK* track(LPCWSTR name, double value, double lo, double hi, double step, LPCWSTR zero_display = nullptr)
	{
		tracks.emplace_back(name, value, lo, hi, step, zero_display);
		items.push_back(&tracks.back());
		return &tracks.back();
	}

	FILTER_ITEM_CHECK* check(LPCWSTR name, bool value)
	{
		checks.emplace_back(name, value);
		items.push_back(&checks.back());
		return &checks.back();
	}

	FILTER_ITEM_SELECT* select(LPCWSTR name, int value, std::vector<FILTER_ITEM_SELECT::ITEM> list)
	{
		list.push_back({ nullptr, 0 });
		lists.push_back(std::move(list));
		selects.emplace_back(name, value, lists.back().data());
		items.push_back(&selects.back());
		return &selects.back();
	}

	void separator(LPCWSTR name)
	{
		separators.emplace_back(name);
		items.push_back(&separators.back());
	}

	void group(LPCWSTR name)
	{
		groups.emplace_back(name);
		items.push_back(&groups.back());
	}

	void hide(LPCWSTR name, LPCWSTR condition_name, FILTER_ITEM_HIDE_RULE::OPERATOR op, int value)
	{
		hides.emplace_back(name, condition_name, op, value);
		items.push_back(&hides.back());
	}
};

//---------------------------------------------------------------------
//	音声チャンクの読み書きと、連続性の確認
//---------------------------------------------------------------------
struct Chunk
{
	int n = 0;
	bool stereo = false;
	std::vector<float> l, r;
	float* left() { return l.data(); }
	float* right() { return stereo ? r.data() : nullptr; }
};

static bool load_chunk(FILTER_PROC_AUDIO* audio, Chunk& chunk)
{
	chunk.n = audio->object->sample_num;
	if (chunk.n <= 0)
	{
		return false;
	}
	chunk.stereo = audio->object->channel_num > 1;
	chunk.l.assign((size_t)chunk.n, 0.0f);
	audio->get_sample_data(chunk.l.data(), 0);
	if (chunk.stereo)
	{
		chunk.r.assign((size_t)chunk.n, 0.0f);
		audio->get_sample_data(chunk.r.data(), 1);
	}
	return true;
}

static void store_chunk(FILTER_PROC_AUDIO* audio, Chunk& chunk)
{
	audio->set_sample_data(chunk.l.data(), 0);
	if (chunk.stereo)
	{
		audio->set_sample_data(chunk.r.data(), 1);
	}
}

// エフェクトのインスタンスごとの状態の共通部分
// 前回の続きのチャンクでなければ(シークなど)、フィルタの内部状態を作り直す
struct StateBase
{
	std::mutex mutex;
	int64_t next_index = -1;
	int sample_rate = 0;

	bool continuous(FILTER_PROC_AUDIO* audio)
	{
		const int64_t index = audio->object->sample_index;
		const int rate = audio->scene->sample_rate;
		const bool result = (index == next_index) && (rate == sample_rate);
		next_index = index + audio->object->sample_num;
		sample_rate = rate;
		return result;
	}
};

template<class S> static void* create_state(int64_t) { return new S(); }
template<class S> static void destroy_state(int64_t, void* state) { delete static_cast<S*>(state); }

template<class S> static S* state_of(FILTER_PROC_AUDIO* audio)
{
	if (audio->userdata)
	{
		return static_cast<S*>(audio->userdata);
	}
	static S fallback;		// FLAG_USERDATA が効かなかった場合の保険
	return &fallback;
}

//=====================================================================
//	1. ノイズ除去
//=====================================================================
struct NoiseFx
{
	EffectDef def;
	FILTER_ITEM_TRACK* reduction, * sensitivity, * attack, * release;
	FILTER_ITEM_SELECT* hum;
};
static NoiseFx* g_noise = nullptr;

struct NoiseState : StateBase
{
	apdsp::NoiseReducer reducer;
};

static bool proc_noise(FILTER_PROC_AUDIO* audio)
{
	Chunk c;
	if (!load_chunk(audio, c))
	{
		return true;
	}
	NoiseState* st = state_of<NoiseState>(audio);
	std::lock_guard<std::mutex> lock(st->mutex);
	if (!st->continuous(audio))
	{
		st->reducer.reset();
	}
	apdsp::NoiseParams p;
	p.reduction_db = g_noise->reduction->value;
	p.sensitivity_db = g_noise->sensitivity->value;
	p.attack_ms = g_noise->attack->value;
	p.release_ms = g_noise->release->value;
	p.hum_hz = g_noise->hum->value;
	st->reducer.process(c.left(), c.right(), c.n, audio->scene->sample_rate, p);
	store_chunk(audio, c);
	return true;
}

//=====================================================================
//	2. イコライザー
//=====================================================================
struct EqFx
{
	EffectDef def;
	FILTER_ITEM_SELECT* preset;
	FILTER_ITEM_TRACK* hp, * lp;
	FILTER_ITEM_TRACK* freq[5], * gain[5], * q[5];	// q[0], q[4] は使わない(シェルフ)
};
static EqFx* g_eq = nullptr;

struct EqState : StateBase
{
	apdsp::Equalizer eq;
};

enum EqPresetId
{
	EQ_CUSTOM = 0,
	EQ_VOICE_CLEAR = 1,
	EQ_VOICE_WARM = 2,
	EQ_BGM_POCKET = 3,
	EQ_TELEPHONE = 4,
	EQ_RADIO = 5,
	EQ_BASS_BOOST = 6,
	EQ_DEMUDDY = 7,
};

static apdsp::EqParams preset_params(int id)
{
	apdsp::EqParams p;
	auto band = [&](int i, double f, double g, double q) { p.freq[i] = f; p.gain[i] = g; p.q[i] = q; };
	switch (id)
	{
	case EQ_VOICE_CLEAR:	// 声をくっきり: 低域の不要な成分を切って、こもり帯域を下げ、明瞭度の帯域を持ち上げる
		p.hp_freq = 80.0;
		band(1, 250.0, -2.0, 1.0);
		band(3, 3500.0, 3.0, 1.0);
		band(4, 9000.0, 2.0, 0.7071);
		break;
	case EQ_VOICE_WARM:		// 声をまろやかに
		p.hp_freq = 70.0;
		band(0, 150.0, 2.0, 0.7071);
		band(3, 4000.0, -2.0, 1.0);
		band(4, 7000.0, -3.0, 0.7071);
		break;
	case EQ_BGM_POCKET:		// BGMの声帯域を空ける (BGMに掛けて、声の邪魔にならないようにする)
		band(2, 1000.0, -2.5, 0.7);
		band(3, 3000.0, -3.5, 0.9);
		break;
	case EQ_TELEPHONE:
		p.hp_freq = 300.0;
		p.lp_freq = 3400.0;
		band(2, 1000.0, 4.0, 1.0);
		break;
	case EQ_RADIO:
		p.hp_freq = 200.0;
		p.lp_freq = 6000.0;
		band(2, 1500.0, 3.0, 0.8);
		band(3, 3500.0, 2.0, 1.0);
		break;
	case EQ_BASS_BOOST:
		band(0, 100.0, 6.0, 0.7071);
		break;
	case EQ_DEMUDDY:		// こもり除去
		band(1, 300.0, -4.0, 1.2);
		band(3, 4000.0, 2.5, 1.0);
		band(4, 10000.0, 2.0, 0.7071);
		break;
	default:
		break;
	}
	return p;
}

static bool proc_eq(FILTER_PROC_AUDIO* audio)
{
	Chunk c;
	if (!load_chunk(audio, c))
	{
		return true;
	}
	EqState* st = state_of<EqState>(audio);
	std::lock_guard<std::mutex> lock(st->mutex);
	if (!st->continuous(audio))
	{
		st->eq.reset();
	}
	apdsp::EqParams p;
	const int preset = g_eq->preset->value;
	if (preset != EQ_CUSTOM)
	{
		p = preset_params(preset);
	}
	else
	{
		p.hp_freq = g_eq->hp->value;
		p.lp_freq = g_eq->lp->value;
		for (int i = 0; i < 5; i++)
		{
			p.freq[i] = g_eq->freq[i]->value;
			p.gain[i] = g_eq->gain[i]->value;
			p.q[i] = g_eq->q[i] ? g_eq->q[i]->value : 0.7071;
		}
	}
	st->eq.configure(p, audio->scene->sample_rate);
	st->eq.process(c.left(), c.right(), c.n);
	store_chunk(audio, c);
	return true;
}

//=====================================================================
//	3. コンプレッサー
//=====================================================================
struct CompFx
{
	EffectDef def;
	FILTER_ITEM_TRACK* threshold, * ratio, * attack, * release, * knee, * makeup, * mix;
	FILTER_ITEM_CHECK* auto_makeup;
};
static CompFx* g_comp = nullptr;

struct CompState : StateBase
{
	apdsp::Compressor comp;
};

static bool proc_comp(FILTER_PROC_AUDIO* audio)
{
	Chunk c;
	if (!load_chunk(audio, c))
	{
		return true;
	}
	CompState* st = state_of<CompState>(audio);
	std::lock_guard<std::mutex> lock(st->mutex);
	if (!st->continuous(audio))
	{
		st->comp.reset();
	}
	apdsp::CompressorParams p;
	p.threshold_db = g_comp->threshold->value;
	p.ratio = g_comp->ratio->value;
	p.attack_ms = g_comp->attack->value;
	p.release_ms = g_comp->release->value;
	p.knee_db = g_comp->knee->value;
	p.makeup_db = g_comp->makeup->value;
	p.auto_makeup = g_comp->auto_makeup->value;
	p.mix = g_comp->mix->value / 100.0;
	st->comp.process(c.left(), c.right(), c.n, audio->scene->sample_rate, p);
	store_chunk(audio, c);
	return true;
}

//=====================================================================
//	オブジェクトのエイリアスから、素材ファイルと再生位置を読む
//	(ラウドネス調整のファイル解析・ダッキングの声量検出で使う)
//=====================================================================
struct MediaSource
{
	std::wstring path;
	double start = 0.0;		// 再生位置(秒)
	double speed = 1.0;		// 再生速度(倍)
};

static std::wstring expand_path_variable(const std::wstring& path)
{
	const std::wstring variable = L"%AppData%";
	if (path.compare(0, variable.size(), variable) == 0 && g_config && g_config->app_data_path)
	{
		return std::wstring(g_config->app_data_path) + path.substr(variable.size());
	}
	return path;
}

static bool is_media_extension(const std::wstring& path)
{
	static const wchar_t* const exts[] = {
		L".wav", L".mp3", L".ogg", L".flac", L".m4a", L".aac", L".wma", L".opus",
		L".mp4", L".mov", L".avi", L".mkv", L".webm", L".wmv", L".flv", L".m4v", L".mpg", L".mpeg", L".mts", L".m2ts" };
	size_t dot = path.find_last_of(L'.');
	if (dot == std::wstring::npos)
	{
		return false;
	}
	std::wstring ext = to_lower(path.substr(dot));
	for (const wchar_t* e : exts)
	{
		if (ext == e)
		{
			return true;
		}
	}
	return false;
}

// 取得できたら true。ファイル項目 "ファイル" が無い(MaterialLibraryの素材など)場合は false
static bool parse_media_source(const char* alias, MediaSource* out)
{
	if (!alias)
	{
		return false;
	}
	MediaSource result;
	bool has_file = false;
	bool has_start = false, has_speed = false;
	const std::string text = alias;
	size_t pos = 0;
	while (pos <= text.size())
	{
		size_t end = text.find('\n', pos);
		const bool last = (end == std::string::npos);
		const std::string line = text.substr(pos, last ? std::string::npos : end - pos);
		const size_t equal = line.find('=');
		if (equal != std::string::npos)
		{
			const std::string key = trim_ascii(line.substr(0, equal));
			const std::string value = trim_ascii(line.substr(equal + 1));
			if (!has_file && key == "\xE3\x83\x95\xE3\x82\xA1\xE3\x82\xA4\xE3\x83\xAB" /* ファイル */ && !value.empty())
			{
				std::wstring path = expand_path_variable(utf8_to_wide(value.data(), value.size()));
				if (is_media_extension(path) && path.find(L'%') == std::wstring::npos)
				{
					result.path = path;
					has_file = true;
				}
			}
			else if (!has_start && key == "\xE5\x86\x8D\xE7\x94\x9F\xE4\xBD\x8D\xE7\xBD\xAE" /* 再生位置 */)
			{
				result.start = std::max(0.0, std::strtod(value.c_str(), nullptr));
				has_start = true;
			}
			else if (!has_speed && key == "\xE5\x86\x8D\xE7\x94\x9F\xE9\x80\x9F\xE5\xBA\xA6" /* 再生速度 */)
			{
				const double percent = std::strtod(value.c_str(), nullptr);
				if (percent > 0.0)
				{
					result.speed = percent / 100.0;
				}
				has_speed = true;
			}
		}
		if (last)
		{
			break;
		}
		pos = end + 1;
	}
	if (!has_file)
	{
		return false;
	}
	*out = result;
	return true;
}

//=====================================================================
//	4. ラウドネス調整
//=====================================================================
struct LoudFx
{
	EffectDef def;
	FILTER_ITEM_SELECT* mode;
	FILTER_ITEM_TRACK* target, * speed, * window, * max_boost, * max_cut, * ceiling;
	FILTER_ITEM_CHECK* use_limiter;
};
static LoudFx* g_loud = nullptr;

enum { LOUD_FOLLOW = 0, LOUD_FILE = 1 };

struct LoudState : StateBase
{
	apdsp::LoudnessFollower follower;
	apdsp::Limiter limiter;
	// ファイル解析の結果
	std::string file_key;
	bool file_ok = false;
	double file_lufs = 0.0;
	bool file_checked = false;
	std::chrono::steady_clock::time_point checked_at;
	bool warned = false;
};

// 解析済みの結果は全インスタンスで共有する
static std::mutex g_lufs_mutex;
static std::map<std::string, std::pair<bool, double>> g_lufs_cache;

static std::string make_lufs_key(const MediaSource& src, double duration, int rate)
{
	char buf[128];
	std::snprintf(buf, sizeof(buf), "|%.4f|%.4f|%d", src.start, duration, rate);
	std::wstring w = src.path + std::wstring(buf, buf + std::strlen(buf));
	int length = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), (int)w.size(), nullptr, 0, nullptr, nullptr);
	std::string key((size_t)std::max(length, 0), '\0');
	if (length > 0)
	{
		WideCharToMultiByte(CP_UTF8, 0, w.c_str(), (int)w.size(), &key[0], length, nullptr, nullptr);
	}
	return key;
}

// ファイルの指定範囲を丸ごと解析して、統合ラウドネス(LUFS)を求める
static bool analyze_file_lufs(const MediaSource& src, double duration_s, int rate, double* lufs)
{
	if (!g_cache || rate <= 0 || duration_s <= 0.0)
	{
		return false;
	}
	apdsp::LoudnessMeter meter;
	meter.init((double)rate);
	const int64_t first = (int64_t)std::floor(src.start * rate);
	const int64_t total = (int64_t)std::ceil(duration_s * rate);
	const int block = rate;		// 1秒ずつ
	std::vector<float> l((size_t)block), r((size_t)block);
	int64_t done = 0;
	int64_t read_total = 0;
	while (done < total)
	{
		const int want = (int)std::min<int64_t>(block, total - done);
		const int got = g_cache->get_audio_file_data(src.path.c_str(), 0, first + done, want, l.data(), r.data());
		if (got <= 0)
		{
			break;
		}
		meter.push(l.data(), r.data(), got);
		read_total += got;
		done += got;
		if (got < want)
		{
			break;
		}
	}
	if (read_total <= 0)
	{
		return false;
	}
	return meter.integrated(lufs);
}

// 現在のオブジェクトのファイル解析結果を更新する (必要な時だけ。解析済みなら共有キャッシュから)
static void refresh_file_lufs(FILTER_PROC_AUDIO* audio, LoudState* st, bool force)
{
	const auto now = std::chrono::steady_clock::now();
	if (!force && st->file_checked && now - st->checked_at < std::chrono::milliseconds(1000))
	{
		return;
	}
	st->file_checked = true;
	st->checked_at = now;

	MediaSource src;
	if (!audio->edit || !audio->edit->get_object_alias || !audio->object->handle ||
		!parse_media_source(audio->edit->get_object_alias(audio->object->handle), &src))
	{
		st->file_ok = false;
		st->file_key.clear();
		return;
	}
	const int rate = audio->scene->sample_rate;
	// タイムライン上の長さ × 再生速度 = ファイルから使う長さ
	const double duration = audio->object->time_total * src.speed;
	const std::string key = make_lufs_key(src, duration, rate);
	if (key == st->file_key)
	{
		return;
	}
	st->file_key = key;

	{
		std::lock_guard<std::mutex> lock(g_lufs_mutex);
		auto it = g_lufs_cache.find(key);
		if (it != g_lufs_cache.end())
		{
			st->file_ok = it->second.first;
			st->file_lufs = it->second.second;
			return;
		}
	}
	double lufs = 0.0;
	const bool ok = analyze_file_lufs(src, duration, rate, &lufs);
	{
		std::lock_guard<std::mutex> lock(g_lufs_mutex);
		if (g_lufs_cache.size() > 256)
		{
			g_lufs_cache.clear();
		}
		g_lufs_cache[key] = { ok, lufs };
	}
	st->file_ok = ok;
	st->file_lufs = lufs;
}

static bool proc_loudness(FILTER_PROC_AUDIO* audio)
{
	Chunk c;
	if (!load_chunk(audio, c))
	{
		return true;
	}
	LoudState* st = state_of<LoudState>(audio);
	std::lock_guard<std::mutex> lock(st->mutex);
	const bool continuous = st->continuous(audio);
	if (!continuous)
	{
		st->follower.reset();
		st->limiter.reset();
	}
	const double rate = audio->scene->sample_rate;
	const double target = g_loud->target->value;
	const double boost = g_loud->max_boost->value;
	const double cut = g_loud->max_cut->value;

	bool done = false;
	if (g_loud->mode->value == LOUD_FILE)
	{
		refresh_file_lufs(audio, st, !continuous);
		if (st->file_ok)
		{
			// ファイル全体の統合ラウドネスから、一定のゲインを掛ける
			const double gain_db = apdsp::clampd(target - st->file_lufs, -cut, boost);
			const float g = (float)apdsp::db2lin(gain_db);
			for (int i = 0; i < c.n; i++)
			{
				c.l[(size_t)i] *= g;
				if (c.stereo) c.r[(size_t)i] *= g;
			}
			done = true;
		}
		else if (!st->warned)
		{
			// ファイルを解析できないオブジェクト (素材ライブラリの素材など) は自動追従で代用する
			st->warned = true;
			log_text(L"AudioPro ラウドネス調整: ファイルを解析できないため、自動追従で処理します");
		}
	}
	if (!done)
	{
		apdsp::FollowerParams p;
		p.target_lufs = target;
		p.window_s = g_loud->window->value;
		p.max_boost_db = boost;
		p.max_cut_db = cut;
		p.speed_db_per_s = g_loud->speed->value;
		st->follower.process(c.left(), c.right(), c.n, rate, p);
	}
	if (g_loud->use_limiter->value)
	{
		st->limiter.process(c.left(), c.right(), c.n, rate, g_loud->ceiling->value, 80.0);
	}
	store_chunk(audio, c);
	return true;
}

//=====================================================================
//	5. ダッキング
//=====================================================================
struct DuckFx
{
	EffectDef def;
	FILTER_ITEM_TRACK* layer_from, * layer_to, * depth, * lookahead, * attack, * hold, * release, * threshold;
	FILTER_ITEM_CHECK* use_level;
};
static DuckFx* g_duck = nullptr;

struct DuckState : StateBase
{
	apdsp::Ducker ducker;
};

// 声のオブジェクトごとの素材情報 (エイリアスの解析は少し重いので、少しの間だけ覚えておく)
struct VoiceSource
{
	bool valid = false;
	MediaSource media;
	double object_start = 0.0;		// タイムライン上の開始時間(秒)
	std::chrono::steady_clock::time_point cached_at;
};
static std::mutex g_voice_mutex;
static std::map<int64_t, VoiceSource> g_voice_sources;

static bool get_voice_source(FILTER_PROC_AUDIO* audio, OBJECT_HANDLE object, MediaSource* media, double* object_start)
{
	const int64_t id = audio->edit->get_object_id(object);
	const auto now = std::chrono::steady_clock::now();
	{
		std::lock_guard<std::mutex> lock(g_voice_mutex);
		auto it = g_voice_sources.find(id);
		if (it != g_voice_sources.end() && now - it->second.cached_at < std::chrono::milliseconds(1000))
		{
			if (!it->second.valid)
			{
				return false;
			}
			*media = it->second.media;
			*object_start = it->second.object_start;
			return true;
		}
	}
	VoiceSource entry;
	entry.cached_at = now;
	if (parse_media_source(audio->edit->get_object_alias(object), &entry.media))
	{
		const OBJECT_LAYER_FRAME frame = audio->edit->get_object_layer_frame(object);
		const double rate = audio->scene->rate;
		const double scale = audio->scene->scale;
		if (rate > 0.0 && scale > 0.0)
		{
			entry.object_start = (double)frame.start * scale / rate;
			entry.valid = true;
		}
	}
	{
		std::lock_guard<std::mutex> lock(g_voice_mutex);
		if (g_voice_sources.size() > 512)
		{
			g_voice_sources.clear();
		}
		g_voice_sources[id] = entry;
	}
	if (!entry.valid)
	{
		return false;
	}
	*media = entry.media;
	*object_start = entry.object_start;
	return true;
}

// 声のファイルの該当位置のレベル(dBFS, RMS)を求める。求められなければ false
static bool voice_level_db(FILTER_PROC_AUDIO* audio, OBJECT_HANDLE object, double timeline_time, double length_s, double* level_db)
{
	if (!g_cache)
	{
		return false;
	}
	MediaSource media;
	double object_start = 0.0;
	if (!get_voice_source(audio, object, &media, &object_start))
	{
		return false;
	}
	const int rate = audio->scene->sample_rate;
	const double file_time = std::max(0.0, media.start + (timeline_time - object_start) * media.speed);
	const int count = std::max(64, (int)(length_s * rate));
	static thread_local std::vector<float> l, r;
	l.assign((size_t)count, 0.0f);
	r.assign((size_t)count, 0.0f);
	const int got = g_cache->get_audio_file_data(media.path.c_str(), 0, (int64_t)std::floor(file_time * rate), count, l.data(), r.data());
	if (got <= 0)
	{
		return false;
	}
	double sum = 0.0;
	for (int i = 0; i < got; i++)
	{
		sum += (double)l[(size_t)i] * l[(size_t)i] + (double)r[(size_t)i] * r[(size_t)i];
	}
	*level_db = 10.0 * std::log10(std::max(sum / (2.0 * got), 1e-20));
	return true;
}

static bool proc_duck(FILTER_PROC_AUDIO* audio)
{
	Chunk c;
	if (!load_chunk(audio, c))
	{
		return true;
	}
	DuckState* st = state_of<DuckState>(audio);
	std::lock_guard<std::mutex> lock(st->mutex);
	if (!st->continuous(audio))
	{
		st->ducker.reset();
	}

	const double rate = audio->scene->sample_rate;
	// UIは1始まり、本体の内部は0始まり
	int layer_a = (int)g_duck->layer_from->value - 1;
	int layer_b = (int)g_duck->layer_to->value - 1;
	if (layer_a > layer_b)
	{
		std::swap(layer_a, layer_b);
	}
	const double lookahead = g_duck->lookahead->value;
	const bool use_level = g_duck->use_level->value;
	const double threshold = g_duck->threshold->value;
	const int own_layer = audio->object->layer;

	// このチャンクの現在時刻 (タイムライン基準)
	const double frame_rate = (audio->scene->scale > 0) ? (double)audio->scene->rate / audio->scene->scale : 30.0;
	const double now_time = (double)audio->object->frame_s / frame_rate + (double)audio->object->sample_index / rate;

	// 10msごとに「声があるか」を調べる
	const int block = std::max(1, (int)(rate * 0.01));
	std::vector<uint8_t> active((size_t)c.n, 0);
	if (audio->edit && audio->get_audio_object)
	{
		for (int b = 0; b < c.n; b += block)
		{
			const int length = std::min(block, c.n - b);
			const double offset = (b + length * 0.5) / rate + lookahead;
			bool voice = false;
			for (int layer = layer_a; layer <= layer_b && !voice; layer++)
			{
				if (layer == own_layer)
				{
					continue;
				}
				OBJECT_HANDLE object = audio->get_audio_object(layer, offset);
				if (!object)
				{
					continue;
				}
				// 音量が0の声は無視する
				OBJECT_AUDIO_PARAM param = {};
				if (audio->get_output_audio_param && audio->get_output_audio_param(object, offset, &param, sizeof(param)) &&
					param.vol_l < 1e-4f && param.vol_r < 1e-4f)
				{
					continue;
				}
				voice = true;
				if (use_level)
				{
					// ファイルから声の大きさを調べられる時だけ、無音(息継ぎ・間)では下げない
					double level = 0.0;
					if (voice_level_db(audio, object, now_time + offset, length / rate, &level) && level < threshold)
					{
						voice = false;
					}
				}
			}
			if (voice)
			{
				std::fill(active.begin() + b, active.begin() + b + length, (uint8_t)1);
			}
		}
	}

	apdsp::DuckParams p;
	p.depth_db = g_duck->depth->value;
	p.attack_ms = g_duck->attack->value;
	p.hold_ms = g_duck->hold->value;
	p.release_ms = g_duck->release->value;
	st->ducker.process(c.left(), c.right(), c.n, rate, active.data(), p);
	store_chunk(audio, c);
	return true;
}

//=====================================================================
//	6. ピッチシフト
//=====================================================================
struct PitchFx
{
	EffectDef def;
	FILTER_ITEM_TRACK* semitones, * window;
};
static PitchFx* g_pitch = nullptr;

struct PitchState : StateBase
{
	apdsp::PitchShifter shifter;
};

static bool proc_pitch(FILTER_PROC_AUDIO* audio)
{
	Chunk c;
	if (!load_chunk(audio, c))
	{
		return true;
	}
	PitchState* st = state_of<PitchState>(audio);
	std::lock_guard<std::mutex> lock(st->mutex);
	if (!st->continuous(audio))
	{
		st->shifter.reset();
	}
	apdsp::PitchParams p;
	p.semitones = g_pitch->semitones->value;
	p.window_ms = g_pitch->window->value;
	st->shifter.process(c.left(), c.right(), c.n, audio->scene->sample_rate, p);
	store_chunk(audio, c);
	return true;
}

//=====================================================================
//	設定項目の定義と登録
//=====================================================================
static void register_effect(HOST_APP_TABLE* host, EffectDef& def, LPCWSTR name, LPCWSTR label,
	bool (*proc)(FILTER_PROC_AUDIO*), void* (*create)(int64_t), void (*destroy)(int64_t, void*))
{
	def.items.push_back(nullptr);
	def.table = {
		FILTER_PLUGIN_TABLE::FLAG_AUDIO | FILTER_PLUGIN_TABLE::FLAG_USERDATA,
		name,
		label,
		PLUGIN_INFO,
		def.items.data(),
		nullptr,		// 画像フィルタなし
		proc,
		create,
		destroy
	};
	host->register_filter_plugin(&def.table);
}

static void build_noise(HOST_APP_TABLE* host)
{
	auto* fx = new NoiseFx();
	g_noise = fx;
	EffectDef& d = fx->def;
	fx->reduction = d.track(L"低減量(dB)", 12.0, 0.0, 40.0, 0.1);
	fx->sensitivity = d.track(L"ノイズ判定の高さ(dB)", 5.0, 0.0, 20.0, 0.1);
	fx->attack = d.track(L"開く速さ(ms)", 5.0, 1.0, 100.0, 1.0);
	fx->release = d.track(L"閉じる速さ(ms)", 120.0, 10.0, 1000.0, 1.0);
	fx->hum = d.select(L"ハムノイズ", 0, { { L"なし", 0 }, { L"50Hz (東日本)", 50 }, { L"60Hz (西日本)", 60 } });
	register_effect(host, d, L"ノイズ除去(AudioPro)", L"音声効果", proc_noise, create_state<NoiseState>, destroy_state<NoiseState>);
}

static void build_eq(HOST_APP_TABLE* host)
{
	auto* fx = new EqFx();
	g_eq = fx;
	EffectDef& d = fx->def;
	for (int i = 0; i < 5; i++) { fx->freq[i] = fx->gain[i] = fx->q[i] = nullptr; }
	fx->preset = d.select(L"プリセット", EQ_CUSTOM, {
		{ L"カスタム (下のバーを使う)", EQ_CUSTOM },
		{ L"声をくっきり", EQ_VOICE_CLEAR },
		{ L"声をまろやかに", EQ_VOICE_WARM },
		{ L"BGMの声帯域を空ける", EQ_BGM_POCKET },
		{ L"電話", EQ_TELEPHONE },
		{ L"ラジオ", EQ_RADIO },
		{ L"低音を強く", EQ_BASS_BOOST },
		{ L"こもり除去", EQ_DEMUDDY },
		});
	d.group(L"カット");
	fx->hp = d.track(L"ローカット(Hz)", 0.0, 0.0, 500.0, 1.0, L"オフ");
	fx->lp = d.track(L"ハイカット(Hz)", 0.0, 0.0, 20000.0, 1.0, L"オフ");
	d.group(L"");
	d.group(L"低域 (シェルフ)");
	fx->freq[0] = d.track(L"低域の周波数(Hz)", 100.0, 20.0, 500.0, 1.0);
	fx->gain[0] = d.track(L"低域ゲイン(dB)", 0.0, -18.0, 18.0, 0.1);
	d.group(L"");
	d.group(L"中低域");
	fx->freq[1] = d.track(L"中低域の周波数(Hz)", 250.0, 50.0, 1000.0, 1.0);
	fx->gain[1] = d.track(L"中低域ゲイン(dB)", 0.0, -18.0, 18.0, 0.1);
	fx->q[1] = d.track(L"中低域のQ", 1.0, 0.1, 10.0, 0.01);
	d.group(L"");
	d.group(L"中域");
	fx->freq[2] = d.track(L"中域の周波数(Hz)", 1000.0, 200.0, 5000.0, 1.0);
	fx->gain[2] = d.track(L"中域ゲイン(dB)", 0.0, -18.0, 18.0, 0.1);
	fx->q[2] = d.track(L"中域のQ", 1.0, 0.1, 10.0, 0.01);
	d.group(L"");
	d.group(L"中高域");
	fx->freq[3] = d.track(L"中高域の周波数(Hz)", 3500.0, 1000.0, 10000.0, 1.0);
	fx->gain[3] = d.track(L"中高域ゲイン(dB)", 0.0, -18.0, 18.0, 0.1);
	fx->q[3] = d.track(L"中高域のQ", 1.0, 0.1, 10.0, 0.01);
	d.group(L"");
	d.group(L"高域 (シェルフ)");
	fx->freq[4] = d.track(L"高域の周波数(Hz)", 8000.0, 2000.0, 20000.0, 1.0);
	fx->gain[4] = d.track(L"高域ゲイン(dB)", 0.0, -18.0, 18.0, 0.1);
	d.group(L"");

	// プリセットを選んだ時は、カスタム用のバーを隠す
	const wchar_t* custom_items[] = {
		L"ローカット(Hz)", L"ハイカット(Hz)",
		L"低域の周波数(Hz)", L"低域ゲイン(dB)",
		L"中低域の周波数(Hz)", L"中低域ゲイン(dB)", L"中低域のQ",
		L"中域の周波数(Hz)", L"中域ゲイン(dB)", L"中域のQ",
		L"中高域の周波数(Hz)", L"中高域ゲイン(dB)", L"中高域のQ",
		L"高域の周波数(Hz)", L"高域ゲイン(dB)" };
	for (const wchar_t* name : custom_items)
	{
		d.hide(name, L"プリセット", FILTER_ITEM_HIDE_RULE::OPERATOR::NOT_EQUAL, EQ_CUSTOM);
	}
	register_effect(host, d, L"イコライザー(AudioPro)", L"音声効果", proc_eq, create_state<EqState>, destroy_state<EqState>);
}

static void build_comp(HOST_APP_TABLE* host)
{
	auto* fx = new CompFx();
	g_comp = fx;
	EffectDef& d = fx->def;
	fx->threshold = d.track(L"しきい値(dB)", -18.0, -60.0, 0.0, 0.1);
	fx->ratio = d.track(L"レシオ", 3.0, 1.0, 20.0, 0.1);
	fx->attack = d.track(L"アタック(ms)", 10.0, 0.1, 200.0, 0.1);
	fx->release = d.track(L"リリース(ms)", 120.0, 5.0, 2000.0, 1.0);
	fx->knee = d.track(L"ニー(dB)", 6.0, 0.0, 24.0, 0.1);
	fx->makeup = d.track(L"メイクアップ(dB)", 0.0, 0.0, 24.0, 0.1);
	fx->auto_makeup = d.check(L"オートメイクアップ", false);
	fx->mix = d.track(L"ミックス(%)", 100.0, 0.0, 100.0, 1.0);
	d.hide(L"メイクアップ(dB)", L"オートメイクアップ", FILTER_ITEM_HIDE_RULE::OPERATOR::EQUAL, 1);
	register_effect(host, d, L"コンプレッサー(AudioPro)", L"音声効果", proc_comp, create_state<CompState>, destroy_state<CompState>);
}

static void build_loudness(HOST_APP_TABLE* host)
{
	auto* fx = new LoudFx();
	g_loud = fx;
	EffectDef& d = fx->def;
	fx->mode = d.select(L"モード", LOUD_FOLLOW, {
		{ L"自動追従 (どの音声でも使える)", LOUD_FOLLOW },
		{ L"ファイル全体を解析して一定ゲイン", LOUD_FILE },
		});
	fx->target = d.track(L"目標ラウドネス(LUFS)", -16.0, -40.0, -6.0, 0.1);
	fx->speed = d.track(L"追従の速さ(dB/秒)", 6.0, 0.5, 30.0, 0.5);
	fx->window = d.track(L"測定の平均化時間(秒)", 3.0, 0.5, 10.0, 0.1);
	fx->max_boost = d.track(L"最大ブースト(dB)", 18.0, 0.0, 40.0, 0.5);
	fx->max_cut = d.track(L"最大カット(dB)", 18.0, 0.0, 40.0, 0.5);
	fx->use_limiter = d.check(L"リミッター (クリップ防止)", true);
	fx->ceiling = d.track(L"天井(dBFS)", -1.0, -12.0, 0.0, 0.1);
	d.hide(L"追従の速さ(dB/秒)", L"モード", FILTER_ITEM_HIDE_RULE::OPERATOR::EQUAL, LOUD_FILE);
	d.hide(L"測定の平均化時間(秒)", L"モード", FILTER_ITEM_HIDE_RULE::OPERATOR::EQUAL, LOUD_FILE);
	d.hide(L"天井(dBFS)", L"リミッター (クリップ防止)", FILTER_ITEM_HIDE_RULE::OPERATOR::EQUAL, 0);
	register_effect(host, d, L"ラウドネス調整(AudioPro)", L"音声効果", proc_loudness, create_state<LoudState>, destroy_state<LoudState>);
}

static void build_duck(HOST_APP_TABLE* host)
{
	auto* fx = new DuckFx();
	g_duck = fx;
	EffectDef& d = fx->def;
	d.separator(L"声のあるレイヤー");
	fx->layer_from = d.track(L"対象レイヤー(開始)", 1.0, 1.0, 100.0, 1.0);
	fx->layer_to = d.track(L"対象レイヤー(終了)", 1.0, 1.0, 100.0, 1.0);
	d.separator(L"下げ方");
	fx->depth = d.track(L"下げる量(dB)", 12.0, 0.0, 48.0, 0.1);
	fx->lookahead = d.track(L"先読み(秒)", 0.15, 0.0, 2.0, 0.01);
	fx->attack = d.track(L"下げる速さ(ms)", 80.0, 1.0, 1000.0, 1.0);
	fx->hold = d.track(L"保持(ms)", 300.0, 0.0, 3000.0, 10.0);
	fx->release = d.track(L"戻る速さ(ms)", 600.0, 10.0, 5000.0, 10.0);
	d.separator(L"声の検出");
	fx->use_level = d.check(L"声の大きさも見る (ファイルを解析)", false);
	fx->threshold = d.track(L"声とみなす音量(dBFS)", -40.0, -80.0, -10.0, 0.5);
	d.hide(L"声とみなす音量(dBFS)", L"声の大きさも見る (ファイルを解析)", FILTER_ITEM_HIDE_RULE::OPERATOR::EQUAL, 0);
	register_effect(host, d, L"ダッキング(AudioPro)", L"音声効果", proc_duck, create_state<DuckState>, destroy_state<DuckState>);
}

static void build_pitch(HOST_APP_TABLE* host)
{
	auto* fx = new PitchFx();
	g_pitch = fx;
	EffectDef& d = fx->def;
	fx->semitones = d.track(L"ピッチ(半音)", 0.0, -24.0, 24.0, 0.01);
	fx->window = d.track(L"窓の長さ(ms)", 50.0, 20.0, 120.0, 1.0);
	register_effect(host, d, L"ピッチシフト(AudioPro)", L"音声効果", proc_pitch, create_state<PitchState>, destroy_state<PitchState>);
}

//---------------------------------------------------------------------
//	汎用プラグイン構造体定義
//---------------------------------------------------------------------
static COMMON_PLUGIN_TABLE g_common_plugin_table = {
	L"AudioPro",
	PLUGIN_INFO
};

//---------------------------------------------------------------------
//	エクスポート関数
//---------------------------------------------------------------------
EXTERN_C __declspec(dllexport) void InitializeLogger(LOG_HANDLE* handle)
{
	g_logger = handle;
}

EXTERN_C __declspec(dllexport) void InitializeConfig(CONFIG_HANDLE* handle)
{
	g_config = handle;
}

EXTERN_C __declspec(dllexport) void InitializeCache(CACHE_HANDLE* handle)
{
	g_cache = handle;
}

EXTERN_C __declspec(dllexport) bool InitializePlugin(DWORD version)
{
	(void)version;
	return true;
}

EXTERN_C __declspec(dllexport) void UninitializePlugin()
{}

EXTERN_C __declspec(dllexport) COMMON_PLUGIN_TABLE* GetCommonPluginTable(void)
{
	return &g_common_plugin_table;
}

EXTERN_C __declspec(dllexport) void RegisterPlugin(HOST_APP_TABLE* host)
{
	build_noise(host);
	build_eq(host);
	build_comp(host);
	build_loudness(host);
	build_duck(host);
	build_pitch(host);
}