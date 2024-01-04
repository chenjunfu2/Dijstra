#include <stddef.h>
#include <map>

template <size_t WpCount>
struct Dijstra
{
	constexpr static const size_t szWpCount = WpCount;
	struct
	{
		size_t szDstWpLong;//到目标的距离
		size_t szPrevWpIdx;//上一个点的索引
	}stWpTable[szWpCount][szWpCount];//stWpTable[Src][Det](Src->Dst)

	//原点集图
	size_t szWpTable[szWpCount][szWpCount];//szWpTable[Src][Det](Src->Dst)
	
	constexpr static const size_t Infinite = (size_t)-1;
	constexpr static const size_t NoPrevWp = (size_t)-1;
public:
	Dijstra(void)
	{
		for (size_t y = 0; y < szWpCount; ++y)
		{
			for (size_t x = 0; x < szWpCount; ++x)
			{
				szWpTable[y][x] = (x == y) ? 0 : Infinite;////斜角0（自己到自己是0），别的都是无限大（默认到不了）
				stWpTable[y][x].szDstWpLong = (x == y) ? 0 : Infinite;//斜角0（自己到自己是0），别的都是无限大（默认到不了）
				stWpTable[y][x].szPrevWpIdx = (x == y) ? x : NoPrevWp;//每个节点的上一个点都默认无点，只有自己能到自己
			}
		}
	}
	~Dijstra(void) = default;

	void CalculateShortestLongAndPath(void)
	{
		for (size_t i = 0; i < szWpCount; ++i)
		{
			IterationDijstraTable(i);
		}
	}

private:
	void IterationDijstraTable(size_t szSrcIdx)
	{
		//默认全没处理过
		bool bIsDstProcessed[szWpCount] = { 0 };

		//循环处理这一排的所有点
		for (size_t i = 0; i < szWpCount; ++i)
		{
			//最小距离点的下标和长度
			size_t szMinLongDstIdx = NoPrevWp, szMinLongDstLong = Infinite;

			//先获取当前未处理过的点集内，距离起始点最小距离的点
			for (size_t szDstIdx = 0; szDstIdx < szWpCount; ++szDstIdx)//szDstIdx是点集中的当前点
			{
				//无需处理已经处理过的点
				if (bIsDstProcessed[szDstIdx])
				{
					continue;
				}

				//获取当前点
				size_t szCurDstLong = szWpTable[szSrcIdx][szDstIdx];

				//如果当前点更小则更新
				if (szCurDstLong < szMinLongDstLong)
				{
					szMinLongDstIdx = szDstIdx;
					szMinLongDstLong = szCurDstLong;
				}
			}

			if (szMinLongDstLong == Infinite)//没有任何一个最小的点了，返回
			{
				return;
			}

			//循环更新当前最小点的邻接点的距离
			for (size_t szDstIdx = 0; szDstIdx < szWpCount; ++szDstIdx)//szDstIdx是当前邻接点
			{
				//无需处理已经处理过的点
				if (bIsDstProcessed[szDstIdx])
				{
					continue;
				}

				//获得[当前最小点到邻接点]的长度
				size_t szNexDstLong = szWpTable[szMinLongDstIdx][szDstIdx];
				if (szNexDstLong == Infinite)//如果存在不可达的点则跳过
				{
					continue;
				}

				//计算[当前起点到当前最小点]+[当前最小点到邻接点]为 [新值]
				size_t szCurDstNewLong = szMinLongDstLong + szNexDstLong;

				//如果[新值]<[当前起点到当前目标]那么代表找到一条比之前更短的路径，则更新当前目标(即为邻接点)
				auto &[szCurDstLong, szCurPrevIdx] = stWpTable[szSrcIdx][szDstIdx];
				if (szCurDstNewLong < szCurDstLong)
				{
					szCurDstLong = szCurDstNewLong;//长度设置成新的长度
					szCurPrevIdx = szMinLongDstIdx;//邻接点上一个位置设置成当前最小点的位置(记录路径)
				}
			}

			//当前处理的最小点计入已处理列表
			bIsDstProcessed[szMinLongDstIdx] = true;
		}
	}

		

};


#include <stdio.h>
#include <iostream>
#include <string>

int mainii(void)
{
	std::string sIpt;

	//建表
	Dijstra<5> dij;

	size_t szIptSrc, szIptDst, szIptLong;
	while (true)
	{
		printf(":");
		std::getline(std::cin, sIpt, '\n');
		int ret = sscanf(sIpt.c_str(), "%zu%zu%zu", &szIptSrc, &szIptDst, &szIptLong);
		if (ret == -1)
		{
			break;
		}
		else if (ret != 3)
		{
			printf("输入错误，请重新输入:\n");
			continue;
		}

		if (szIptSrc < 0 || szIptSrc >= dij.szWpCount ||
			szIptDst < 0 || szIptDst >= dij.szWpCount)
		{
			printf("下标超过范围，请重新输入:\n");
			continue;
		}
		if (szIptSrc == szIptDst)
		{
			printf("两个位置相同，请重新输入:\n");
			continue;
		}

		dij.szWpTable[szIptSrc][szIptDst] = dij.szWpTable[szIptDst][szIptSrc] = szIptLong;//因为是无向图所以双向通路
	}

	//迭代计算最短长度和路径
	dij.CalculateShortestLongAndPath();


	while (true)
	{
		printf("输入起始点和目的地\n");
		std::getline(std::cin, sIpt, '\n');
		int ret = sscanf(sIpt.c_str(), "%zu%zu", &szIptSrc, &szIptDst);
		if (ret == -1)
		{
			break;
		}
		else if (ret != 2)
		{
			printf("输入错误，请重新输入:\n");
			continue;
		}

		if (szIptSrc < 0 || szIptSrc >= dij.szWpCount ||
			szIptDst < 0 || szIptDst >= dij.szWpCount)
		{
			printf("下标超过范围，请重新输入:\n");
			continue;
		}

		size_t szLong = dij.stWpTable[szIptSrc][szIptDst].szDstWpLong;
		if (szLong == dij.Infinite)
		{
			printf("长度为:无限\n");
		}
		else
		{
			printf("长度为:%zu\n", szLong);
		}

		size_t szIdx = szIptDst;
		while (szIdx != szIptSrc && szIdx != dij.NoPrevWp)
		{
			printf("%zu<-", szIdx);
			szIdx = dij.stWpTable[szIptSrc][szIdx].szPrevWpIdx;
		}
		printf("%zu\n", szIdx);
	}

	return 0;
}

/*
0 1 20
1 2 30
2 3 50
3 0 40
2 4 60
3 4 70
*/

