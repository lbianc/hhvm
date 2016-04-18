/*
   +----------------------------------------------------------------------+
   | HipHop for PHP                                                       |
   +----------------------------------------------------------------------+
   | Copyright (c) 2010-2016 Facebook, Inc. (http://www.facebook.com)     |
   +----------------------------------------------------------------------+
   | This source file is subject to version 3.01 of the PHP license,      |
   | that is bundled with this package in the file LICENSE, and is        |
   | available through the world-wide-web at the following url:           |
   | http://www.php.net/license/3_01.txt                                  |
   | If you did not receive a copy of the PHP license and are unable to   |
   | obtain it through the world-wide-web, please send a note to          |
   | license@php.net so we can mail you a copy immediately.               |
   +----------------------------------------------------------------------+
*/

#include "hphp/runtime/vm/jit/mixed-array-offset-profile.h"

#include "hphp/runtime/base/array-data.h"
#include "hphp/runtime/base/mixed-array.h"
#include "hphp/runtime/base/mixed-array-defs.h"
#include "hphp/runtime/base/string-data.h"
#include "hphp/runtime/vm/member-operations.h"

#include "hphp/util/safe-cast.h"

#include <folly/Optional.h>

#include <algorithm>
#include <cstring>

namespace HPHP { namespace jit {

///////////////////////////////////////////////////////////////////////////////

folly::Optional<uint32_t>
MixedArrayOffsetProfile::choose() const {
  Line hottest;
  uint32_t total = 0;

  if (!m_init) return folly::none;

  for (auto i = 0; i < kNumTrackedSamples; ++i) {
    auto const& line = m_hits[i];
    if (!validPos(line.pos)) break;

    if (line.count > hottest.count) hottest = line;
    total += line.count;
  }
  total += m_untracked;

  return hottest.count * 10 >= total * 8
    ? folly::make_optional(safe_cast<uint32_t>(hottest.pos))
    : folly::none;
}

bool MixedArrayOffsetProfile::update(int32_t pos, uint32_t count) {
  if (!m_init) init();

  if (!validPos(pos)) {
    m_untracked += count;
    return false;
  }

  for (auto i = 0; i < kNumTrackedSamples; ++i) {
    auto& line = m_hits[i];

    if (line.pos == pos) {
      line.count += count;
      return true;
    }
    if (line.pos == -1) {
      line.pos = pos;
      line.count = count;
      return true;
    }
  }

  m_untracked += count;
  return false;
}

void MixedArrayOffsetProfile::update(const ArrayData* ad, int64_t i) {
  auto const pos = ad->isMixed()
    ? MixedArray::asMixed(ad)->find(i)
    : -1;
  update(pos, 1);
}

void MixedArrayOffsetProfile::update(const ArrayData* ad,
                                     const StringData* sd,
                                     bool checkForInt) {
  auto const pos = [&]() -> int32_t {
    if (!ad->isMixed()) return -1;
    auto const a = MixedArray::asMixed(ad);

    int64_t i;
    return checkForInt && ad->convertKey(sd, i)
      ? a->find(i)
      : a->find(sd, sd->hash());
  }();
  update(pos, 1);
}

void MixedArrayOffsetProfile::reduce(MixedArrayOffsetProfile& l,
                                     const MixedArrayOffsetProfile& r) {
  if (!r.m_init) return;

  Line scratch[2 * kNumTrackedSamples];
  auto n = uint32_t{0};

  for (auto i = 0; i < kNumTrackedSamples; ++i) {
    auto const& rline = r.m_hits[i];
    if (!validPos(rline.pos)) break;

    // Update `l'.  If `l' can't record the update, save it to scratch.
    if (!l.update(rline.pos, rline.count)) {
      scratch[n++] = rline;
    }
  }
  l.m_untracked += r.m_untracked;

  if (n == 0) return;
  assertx(n <= kNumTrackedSamples);

  // Sort the combined samples, then copy the top hits back into `l'.
  std::memcpy(&scratch[n], &l.m_hits[0],
              kNumTrackedSamples * sizeof(Line));
  std::sort(&scratch[0], &scratch[n + kNumTrackedSamples],
            [] (Line a, Line b) { return a.count > b.count; });
  std::memcpy(l.m_hits, scratch, kNumTrackedSamples);

  // Add the hits in the discarded tail to m_untracked.
  for (auto i = kNumTrackedSamples; i < n + kNumTrackedSamples; ++i) {
    auto const& line = scratch[i];
    if (!validPos(line.pos)) break;

    l.m_untracked += line.count;
  }
}

void MixedArrayOffsetProfile::init() {
  assertx(!m_init);
  for (auto i = 0; i < kNumTrackedSamples; ++i) {
    m_hits[i] = Line{};
  }
  m_init = true;
}

///////////////////////////////////////////////////////////////////////////////

}}
