#include <iostream>

class Abstract {
public:
  virtual void fromBase() = 0;
  virtual void toBase() = 0;
};

class Terminate : public Abstract {
public:
  virtual void fromBase() override { std::cout << "Terminate::fromBase()" << std::endl; }
  virtual void toBase() override { std::cout << "Terminate::toBase()" << std::endl; }
};

template < typename Base = Terminate > class ChainA : public Base {
public:
  virtual void fromBase() override {
    Base::fromBase();
    std::cout << "ChainA::fromBase()" << std::endl;
  }

  virtual void toBase() override {
    std::cout << "ChainA::toBase()" << std::endl;
    Base::toBase();
  }
};

template < typename Base = Terminate > class ChainB : public Base {
public:
  virtual void fromBase() override {
    Base::fromBase();
    std::cout << "ChainB::fromBase()" << std::endl;
  }

  virtual void toBase() override {
    std::cout << "ChainB::toBase()" << std::endl;
    Base::toBase();
  }
};

template < typename Base = Terminate > class ChainC : public Base {
public:
  virtual void fromBase() override {
    Base::fromBase();
    std::cout << "ChainC::fromBase()" << std::endl;
  }

  virtual void toBase() override {
    std::cout << "ChainC::toBase()" << std::endl;
    Base::toBase();
  }
};

int main(int argc, char *argv[]) {
  ChainC< ChainB< ChainA<> > > chain;
  chain.fromBase();
  chain.toBase();
  return 0;
}