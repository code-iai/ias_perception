#include "AlgorithmSelector.h"
#include "XMLTag.h"


template<typename T>
AlgorithmSelector<T>::AlgorithmSelector (XMLTag* node
#ifdef LOGFILE
                                         , LogFile& log
#endif /*LOGFILE*/
                                         ):
          m_tree(NULL),
          m_paLength(0),
          m_paAllocatedLength(0),
          m_paDim(4)
#ifdef LOGFILE
    ,m_logfile(log)
#endif /*LOGFILE*/

{
  m_mutexAlgEval = new boost::mutex();
  if(node != NULL)
  {
    try
    {
      m_algorithmlist = XMLTag::Load(node, &m_algorithmlist);
    }
    catch(char* pst)
    {
      printf("Error loading Algorithm list: %s\n", pst);
    }
    AlgorithmID_t id = 0;
    typename std::vector< AlgorithmEval<T> > mv = GetAlgorithmList(&mv);
    for(typename std::vector< class AlgorithmEval<T> >::const_iterator iter = mv.begin();
      iter != mv.end(); iter++)
    {
      (*iter).m_algorithm->m_AlgIndex = id;
      id++;
    }
  }
}

//
// Methods
//
template <typename T>
XMLTag* AlgorithmSelector<T>::Save(std::string name )
{
  return XMLTag::Tag(m_algorithmlist, name);
}

template <typename T>
int AlgorithmSelector<T>::InsertInList(AlgorithmEval<T> eval)
{
  m_algorithmlist.push_back(eval);
  eval.m_algorithm->m_AlgIndex = m_algorithmlist.size() - 1;
  return (int)m_algorithmlist.size() - 1;
}

template <typename T>
std::vector<AlgorithmEval<T> >& AlgorithmSelector<T>::GetAlgorithmList(std::vector<AlgorithmEval<T> >* )
{
  return m_algorithmlist;
}

template<typename T>
Algorithm<T>* AlgorithmSelector<T>::getAlgorithm(int index)
{
  return m_algorithmlist[index].m_algorithm;
}


template<typename T>
Algorithm<T>* AlgorithmSelector<T>::BestAlgorithm(int type, const Signature &sig, const std::vector<Sensor*> &sensors)
{
  Algorithm<T>* selAlgorithm = NULL;
  double maxEval = -1.0;
  typename std::vector< AlgorithmEval<T> > mv = GetAlgorithmList(&mv);
  for(typename std::vector< class AlgorithmEval<T> >::const_iterator iter = mv.begin();
    iter != mv.end(); iter++)
  {
    double sig_compatibility = (*iter).m_algorithm->CheckSignature(sig, sensors);
    if(CheckTypeCompatibility((*iter).m_algorithmType, type) && sig_compatibility > 0.0)
    {
      double earlierExperiences = 0.0;
      std::pair<double, int> pp = (*iter).GetEval(sig.m_ID);
      if(pp.second > 0)
      {
        earlierExperiences = pp.first / pp.second;
        printf("%s: Earlier experience for this object ID (%f, %d)\n",(*iter).m_algorithm->GetName().c_str(), earlierExperiences, pp.second);
        if(earlierExperiences == 0.0)
        {
          ROS_WARN("Reject to perform action on object ID %ld with alg %s", sig.m_ID,  (*iter).m_algorithm->GetName().c_str());
        }
      }
      else
        printf("%s: No earlier experience for this object ID %ld (map entries: %ld)\n", (*iter).m_algorithm->GetName().c_str(), sig.m_ID,(*iter).MapLength());

      double currEval = (min(1.0, (*iter).m_eval * sig_compatibility) + earlierExperiences*pp.second) / (1+pp.second);
      if(currEval > maxEval)
      {

        maxEval = currEval;
        selAlgorithm = (*iter).m_algorithm;
#ifdef _DEBUG
        printf("The algorithm %s has a good probability for the signature (%f)\n", (*iter).m_algorithm->GetName().c_str(), maxEval);
#endif
      }
      else
      {
#ifdef _DEBUG
        printf("The algorithm %s is  worse than others (%f < %f)\n", (*iter).m_algorithm->GetName().c_str(), currEval, maxEval);
#endif
      }
    }
  }
  return selAlgorithm;
}

template<typename T>
std::vector<Algorithm<T>*> AlgorithmSelector<T>::BestAlgorithmList(int type, const Signature &sig, const std::vector<Sensor*> &sensors)
{
  std::vector<Algorithm<T>*> selAlgorithm;
  double maxEval = -1.0;
  typename std::vector< AlgorithmEval<T> > mv = GetAlgorithmList(&mv);
  for(typename std::vector< class AlgorithmEval<T> >::const_iterator iter = mv.begin();
    iter != mv.end(); iter++)
  {
    double sig_compatibility = (*iter).m_algorithm->CheckSignature(sig, sensors);
    if(CheckTypeCompatibility((*iter).m_algorithmType, type) && sig_compatibility > 0.0)
    {
      double earlierExperiences = 0.0;

      std::pair<double, int> pp = (*iter).GetEval(sig.m_ID);
      if(pp.second > 0)
      {
        earlierExperiences = pp.first / pp.second;
        printf("%s: Earlier experience for this object ID (%f)\n",(*iter).m_algorithm->GetName().c_str(), earlierExperiences);
        if(earlierExperiences == 0.0)
        {
          printf("Reject to go with this object ID\n");
          ROS_WARN("Reject to perform action on object ID %ld with alg %s", sig.m_ID,  (*iter).m_algorithm->GetName().c_str());
        }
      }
      else
        printf("%s: No earlier experience for this object ID %ld (map entries: %ld)\n", (*iter).m_algorithm->GetName().c_str(), sig.m_ID,(*iter).MapLength());

      double currEval = (min(1.0, (*iter).m_eval * sig_compatibility) + earlierExperiences) / 2.0;
      if(currEval > 0.0)
      {
        if(currEval > maxEval)
          maxEval = currEval;
        selAlgorithm.push_back((*iter).m_algorithm);
#ifdef _DEBUG
        printf("The algorithm %s has a good probability for the signature (%f)\n", (*iter).m_algorithm->GetName().c_str(), currEval);
#endif
      }
      else
      {
#ifdef _DEBUG
        printf("The algorithm %s is  worse than others (%f < %f)\n", (*iter).m_algorithm->GetName().c_str(), currEval, maxEval);
#endif
      }
    }
  }
  return selAlgorithm;
}


template<typename T>
void AlgorithmSelector<T>::EvalAlgorithm(Evaluable* alg, double eval, double time, Signature* relatedElem)
{
  lock lk (*m_mutexAlgEval);
  typename std::vector<AlgorithmEval<T> > helper;
  typename std::vector<AlgorithmEval<T> > &mv = GetAlgorithmList(&helper);
  for(typename std::vector<AlgorithmEval<T> >::iterator iter = mv.begin();
    iter != mv.end(); iter++)
  {

    if((*iter).m_algorithm == alg)
    {
#ifdef _DEBUG
      printf("Add Evaluation to Algorithm %s: %f\n",alg->GetName().c_str(), eval);
#endif
      (*iter).m_eval = ((*iter).m_eval + eval) / 2;    //TODO eval rel situation and object...
      (*iter).m_avgRunTime = ((*iter).m_avgRunTime + time) / 2;

      if(relatedElem != NULL)
      {
        (*iter).SetEval(relatedElem->m_ID, eval);

        printf("Added %ld to the list of length (now %ld) of objects of %s\n", relatedElem->m_ID, (*iter).MapLength(), (*iter).m_algorithm->GetName().c_str());
        printf("EvalAlgorithm: Do we have a pose?\n");
        RelPose* pose_t = relatedElem->GetObjectPose();
        if(pose_t != NULL)
        {
          try
          {
            Matrix m = pose_t->GetMatrix(ID_WORLD);
            /**TODO  add position to ANN search tree,
                   recalculate search tree
            */
            printf("EvalAlgorithm: Taking pose %ld of elem %ld\n", pose_t->m_uniqueID, relatedElem->m_ID);
            m_paLength++;
            if(m_tree != NULL)
            {
              delete m_tree;
            }

            if(m_paLength > m_paAllocatedLength)
            {
              bool copy = true;
              if(m_paAllocatedLength == 0)
                copy = false;
              m_paAllocatedLength += 100;
              m_paDim = 4;
              ANNpointArray newPA = annAllocPts(m_paAllocatedLength, m_paDim);
              double*   newEval = new double[m_paAllocatedLength];
              if(copy)
              {
                printf("EvalAlgorithm: We have to copy old data\n");
                for(int i = 0; i < (m_paLength - 1); i++)
                {
                  memcpy(newPA[i], m_pointArray[i],sizeof(ANNcoord) * m_paDim);
                }
                memcpy(newEval, m_pointEval,sizeof(double)*(m_paLength - 1));
                annDeallocPts(m_pointArray);
                delete m_pointEval;
              }
              m_pointArray = newPA;
              m_pointEval = newEval;
            }
            m_pointArray[(m_paLength - 1)][0] = (ANNcoord)(*iter).m_algorithm->m_AlgIndex;
            m_pointArray[(m_paLength - 1)][1] = (ANNcoord)m.element(0,3);
            m_pointArray[(m_paLength - 1)][2] = (ANNcoord)m.element(1,3);
            m_pointArray[(m_paLength - 1)][3] = (ANNcoord)m.element(2,3);
            m_pointEval[m_paLength - 1] = eval;
            m_tree = new ANNkd_tree((ANNpointArray)m_pointArray, m_paLength, m_paDim);
            printf("EvalAlgorithm: Added point [ %f ,  %f,  %f,  %f ]\n",  m_pointArray[(m_paLength - 1)][0], m_pointArray[(m_paLength - 1)][1],
            m_pointArray[(m_paLength - 1)][2], m_pointArray[(m_paLength - 1)][3]);

          }
          catch(const char* text)
          {
            printf("Error: %s\n", text);
          }

        }
      }

#ifdef LOGFILE
      m_logfile.Log(alg->GetName(), GetName(), time / 100000000 , eval, relatedElem);
#endif /*LOGFILE*/
      break;
    }
  }
}

template<typename T>
double AlgorithmSelector<T>::ReEvalKernel(double val)
{
  return 2.0 / (1.0 + exp(-val));
}

template<typename T>
void AlgorithmSelector<T>::ReevaluatePose(Algorithm<T>* alg, RelPose*& pose)
{
  lock lk(*m_mutexAlgEval);
  Matrix m = pose->GetMatrix(ID_WORLD);
  ANNcoord point[4];
  point[0] = (ANNcoord)alg->m_AlgIndex;
  point[1] = (ANNcoord)m.element(0,3);
  point[2] = (ANNcoord)m.element(1,3);
  point[3] = (ANNcoord)m.element(2,3);
  ANNidx idx[4];
  ANNdist dist[4];

  printf("ReevaluatePose %s %ld (number of entries %d, p: [%f %f %f %f ] (qual %f))\n", alg->GetName().c_str(), pose->m_uniqueID, m_paLength, point[0], point[1], point[2], point[3], pose->m_qualityMeasure);

  if(m_paLength > 4)
  {
    double test;
    m_tree->annkSearch(point, 4, idx, dist, 0.0);
    if((dist[0] + dist[1] + dist[2] + dist[3]) > 0.0001)
    {
      test = (m_pointEval[idx[0]] * ReEvalKernel(dist[0]) +
      m_pointEval[idx[1]] * ReEvalKernel(dist[1]) +
      m_pointEval[idx[2]] * ReEvalKernel(dist[2]) +
      m_pointEval[idx[3]] * ReEvalKernel(dist[3])) / (4*(dist[0] + dist[1] + dist[2] + dist[3]));
    }
    else
    {
      test = (m_pointEval[idx[0]] * ReEvalKernel(dist[0]) +
      m_pointEval[idx[1]] * ReEvalKernel(dist[1]) +
      m_pointEval[idx[2]] * ReEvalKernel(dist[2]) +
      m_pointEval[idx[3]] * ReEvalKernel(dist[3])) / 4;
    }

    double test2 = ReEvalKernel(dist[0]);
    printf("Found 4 neighnors: Eval before %f\n", pose->m_qualityMeasure);
    //pose->m_qualityMeasure = (pose->m_qualityMeasure + test) / 2;
    printf("Eval after                     %f\n", pose->m_qualityMeasure);
    printf("test2(%f)                %f\n", dist[0], test2);
    printf("test                           %f\n", test);
  }
}

template<typename T>
bool AlgorithmSelector<T>::CheckTypeCompatibility(int listedType, int askedType)
{
  if(askedType == listedType)
    return true;
  if(askedType / 0x100 == listedType / 0x100)
  {
    if(askedType % 0x100 <= listedType % 0x100)
      return true;
  }
  return false;
}

template<typename T>
int AlgorithmSelector<T>::AddAlgorithm(Algorithm<T>* alg, int nType, double dEval, double dTime)
{
  if(dEval <= -1.0)
    dEval = -0.99;
  if(alg == NULL)
    return -1;
  AlgorithmEval<T> pair(alg, nType, dEval, dTime);
  return InsertInList(pair);
}
