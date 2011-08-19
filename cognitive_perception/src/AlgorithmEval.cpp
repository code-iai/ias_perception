/*
 * Copyright (C) 2009 by Ulrich Friedrich Klank <klank@in.tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "AlgorithmEval.h"
#include "XMLTag.h"


using namespace cop;


template<typename T>
AlgorithmEval<T>::AlgorithmEval(XMLTag* tag)
{
    XMLTag* nodealgType = tag->GetChild(XML_NODE_ALGTYPE);
    if(nodealgType != NULL)
    {
        m_algorithmType = XMLTag::Load(nodealgType, &m_algorithmType);
        m_eval = XMLTag::Load(tag->GetChild(XML_NODE_EVAL), &m_eval);
        m_avgRunTime = XMLTag::Load(tag->GetChild(XML_NODE_AVGTIME), &m_avgRunTime);
        m_algorithm = XMLTag::Load(tag->GetChild(0), &m_algorithm);
    }
    else
    {
        printf("Deprecated XML entry\n");
        std::pair < Algorithm<std::vector<RelPose*> >*, std::pair<int, std::pair< double , double > > > loader;
        loader = XMLTag::Load(tag, &loader);
        m_algorithm = (Algorithm<T>*)loader.first;
        m_algorithmType = loader.second.first;
        m_eval = loader.second.second.first;
        m_avgRunTime = loader.second.second.second;
    }

    XMLTag* nodeMap = tag->GetChild(XML_NODE_OBJECTEVALMAP);
    if(nodeMap != NULL)
      m_objectEval = XMLTag::Load(nodeMap, &m_objectEval);
    if(m_algorithm == NULL)
    {
       if(tag != NULL && tag->GetChild(0) != NULL)
       {
         ROS_WARN("AlgorithmEval::AlgorithmEval XMLFile contains an unknown Algorithm: %s\n", tag->GetChild(0)->GetName().c_str());
       }
       else if(tag != NULL)
       {
         printf("Node: %s\n", tag->GetName().c_str());
       }
       throw "Algorithm not implemented";
    }
}

template<typename T>
XMLTag* AlgorithmEval<T>::Save(std::string name)
{
    XMLTag* tag = new XMLTag(name.compare("") == 0 ? XML_NODE_ALGORITHMEVAL : name);
    tag->AddChild(m_algorithm->Save());
    tag->AddChild(XMLTag::Tag(m_algorithmType, XML_NODE_ALGTYPE));
    tag->AddChild(XMLTag::Tag(m_eval, XML_NODE_EVAL));
    tag->AddChild(XMLTag::Tag(m_avgRunTime, XML_NODE_AVGTIME));
    tag->AddChild(XMLTag::Tag(m_objectEval, XML_NODE_OBJECTEVALMAP));
    return tag;
}

#ifndef WIN32
template class AlgorithmEval<std::vector<Signature* > >;
template class AlgorithmEval<std::vector<RelPose* > >;
template class AlgorithmEval<Descriptor*>;
template class AlgorithmEval<ImprovedPose>;
template class AlgorithmEval<double>;
#else
template AlgorithmEval<std::vector<Signature* > >;
template AlgorithmEval<std::vector<RelPose* > >;
template AlgorithmEval<Descriptor*>;
template AlgorithmEval<ImprovedPose>;
template AlgorithmEval<double>;
#endif
