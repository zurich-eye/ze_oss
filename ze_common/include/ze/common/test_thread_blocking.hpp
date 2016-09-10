// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

namespace ze {

//! Helper class to perform blocking tests.
class BlockingTest
{
public:
  BlockingTest() = default;
  virtual ~BlockingTest() = default;
  virtual void performBlockingAction(unsigned testId) = 0;
  virtual void performUnblockingAction(unsigned testId) = 0;
  void runBlockingTest(unsigned testId, unsigned timeout);
  unsigned testId() const { return test_id_; }
  void setFinished() { finished_ = true; }

private:
  void runThread();
  unsigned test_id_ = 0;
  bool finished_ = false;
};

} // namespace ze
