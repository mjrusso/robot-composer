#include <iostream>

using namespace std;


template<int HSIZE>
class SensorHistory {
public:
  int datapts;
  int data[HSIZE];
  int wpos;
  int rpos;

  SensorHistory() {
    this->datapts = 0;
    this->wpos = 0;
    this->rpos = 0;
  };

  inline bool ready() {
    return this->datapts >= HSIZE;
  };

  inline int next_position(int currentPos) {
    return (currentPos + 1) % HSIZE;
  };
  
  inline void add_point(int data) {
    if (!this->ready()) {
        this->datapts += 1;
    } else {
        this->rpos = next_position(this->rpos);
    }
    (this->data)[this->wpos] = data;
    this->wpos = next_position(this->wpos);
  };

  // 
  inline void get_data(int* outbuf) {
    int* outp = outbuf;
    for (int* intp = this->data + this->rpos,
            * endp = this->data + HSIZE; intp < endp;)
            {
               *outp++ = *intp++;
            }
    for (int* intp = this->data,
            * endp = this->data + this->rpos;
            intp <  endp; )
        {
            *outp++ = *intp++;
        }
  };


 /// Debugging methods
   inline void dump_buffer(int* outbuf) {
    for (int* outi = outbuf, c = 0; c < HSIZE; ++c, ++outi) {
        *outi = this->data[c];
    } 
   };   
    

   inline void dump_buffer_o(int* outbuf) {
     for (int* outi = outbuf, c = 0, r = this->rpos; c < HSIZE; ++c, ++outi, r = next_position(r)) {
       *outi = this->data[r];
      } 
    }
};


int do_get_avg_reading(int* buf, int size) {
    // Gets an average reading for sensors
};

template<int HSIZE>
class AveragingSensor {
public:
    SensorHistory<HSIZE>& m_hist;

    AveragingSensor(SensorHistory<HSIZE>& hist)
    : m_hist(hist)
    {};

    inline int get_average() {
        int baseline = m_hist.data[0];
        if (m_hist.ready()) {
            int delta_sum = 0;
            for (int i = 1; i < HSIZE; ++i) {
               delta_sum += (m_hist.data[i] - baseline);
            }
            return baseline + delta_sum / HSIZE;
        } else {
            return baseline;
        }
    };
};

template<int HSIZE>
class SensorChangeTrigger {
private:
    int m_dir;
    int m_pcdiff;
    SensorHistory<HSIZE> &m_hist;

public:
    SensorChangeTrigger(int direction, int pcdiff, SensorHistory<HSIZE> &hist)
    : m_dir(direction), m_pcdiff(pcdiff), m_hist(hist)
    {
        
    };

    inline void get_delta_averages(int& out_prev, int& out_next) {
        int buf[HSIZE];
        m_hist.get_data(buf);
        {
           int base_prev = buf[0];
           int base_next = buf[HSIZE/2];
           int deltas_prev = 0;
           int deltas_next = 0;
           for (int i = 0; i < HSIZE/2; ++i) {
                deltas_prev += (buf[i] - base_prev);
            }
          out_prev = base_prev + deltas_prev / (HSIZE/2);
          for (int i = HSIZE/2; i <HSIZE; ++i) {
                deltas_next += (buf[i] - base_next);
           }
          out_next = base_next + deltas_next / (HSIZE - HSIZE/2);
        }
    };

    inline bool is_active() {
        //1. Compute averages over half-ranges
        int avg_prev = 0;
        int avg_next = 0;
        get_delta_averages(avg_prev, avg_next);
        //2. Make triggering decisions!
        //2.1 Check percentage threshold
        int delta_avgs = avg_next - avg_prev;
        if ((delta_avgs * 100) >= (avg_prev * m_pcdiff)) {
            //2.2 Check direction
            if (0 != m_dir) {
                return ((delta_avgs > 0) && (m_dir > 0))
                        || ((delta_avgs < 0) && (m_dir < 0));
            } else {
                return true;
            }
        }
        return false;
    };
};


int dump_buf(int size, std::ostream& whatev, int* buf) {
    for (int i = 0; i < size; ++i) {
        whatev << buf[i] << "  ";
    }
    return 0;
};

int main_sensorh(int argc, char** argv) {
    std::cout << "This is a test!" << std::endl;
    int dbugb[5];
    SensorHistory<5> hist;
    std::cout << "1. " << hist.ready() << std::endl;
    for (int i = 0; i < 5; ++i) {
        hist.add_point(i+1);
    }
    std::cout << "2. " << hist.ready() << std::endl;
    hist.dump_buffer(dbugb);
    std::cout << "3.a "; dump_buf(5, std::cout, dbugb); std::cout << std::endl;
    hist.dump_buffer_o(dbugb);
    std::cout << "3.b "; dump_buf(5, std::cout, dbugb); std::cout << std::endl;
    hist.get_data(dbugb);
    std::cout << "3.c "; dump_buf(5, std::cout, dbugb); std::cout << std::endl;
    
    hist.add_point(6);
    hist.add_point(7);
    hist.dump_buffer(dbugb);
    std::cout << "4.a "; dump_buf(5, std::cout, dbugb); std::cout << std::endl;
    hist.dump_buffer_o(dbugb);
    std::cout << "4.b "; dump_buf(5, std::cout, dbugb); std::cout << std::endl;
    hist.get_data(dbugb);
    std::cout << "4.c "; dump_buf(5, std::cout, dbugb); std::cout << std::endl;


    AveragingSensor<5> savg(hist);
    std::cout << "5.a "; std::cout << savg.get_average() << std::endl;
 
    { //Test bidirectional 10% state change trigger
        SensorChangeTrigger<5> schg(0, 10, hist);
        std::cout << "6.a "; std::cout << schg.is_active() << std::endl;
    }
    { // Test negative 10% state change trigger
        SensorChangeTrigger<5> schg(-1, 10, hist);
        std::cout << "6.b "; std::cout << schg.is_active() << std::endl;
    }
    { // Test positive 200% state change trigger
        SensorChangeTrigger<5> schg(+1, 200, hist);
        std::cout << "6.c "; std::cout << schg.is_active() << std::endl;
    }
    { // Test positive 20% state change trigger
        SensorChangeTrigger<5> schg(+1, 20, hist);
        std::cout << "6.d "; std::cout << schg.is_active() << std::endl;
    }
    
};

int main(int argc, char** argv) {
    std::cout << "This is UBER test" << std::endl;
    main_sensorh(argc, argv);
    return 0;
};
