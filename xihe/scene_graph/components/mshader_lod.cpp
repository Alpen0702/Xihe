#include "mshader_lod.h"

namespace xihe::sg
{

// triangle_count和triangle_offset有点坑。现在triangle_count视为三角形数量，但triangle_offset视为索引的偏移，即三角形偏移量的三倍。
// 原本xihe使用的时三个uint8的index压成一个uint32，现在改为每个index都是uint32。

// 把低级别的meshlet分组，每个组包含多个meshlet，相当于比原先meshlet更高一级的cluster。后续会用meshopt_simplify来简化cluster
static std::vector<MeshletGroup> groupMeshlets(MeshPrimitiveData &primitive, std::span<Meshlet> meshlets)
{
	// ===== Build meshlet connections
	auto groupWithAllMeshlets = [&]() {
		MeshletGroup group;
		for (int i = 0; i < meshlets.size(); ++i)
		{
			group.meshlets.push_back(i);
		}
		return std::vector{group};
	};
	if (meshlets.size() < 16)
	{
		return groupWithAllMeshlets();
	}

	// meshlets represented by their index into 'meshlets'
	std::unordered_map<MeshletEdge, std::vector<std::size_t>, MeshletEdgeHasher> edges2Meshlets;
	std::unordered_map<std::size_t, std::vector<MeshletEdge>>                    meshlets2Edges;        // probably could be a vector

	// for each meshlet
	for (std::size_t meshletIndex = 0; meshletIndex < meshlets.size(); meshletIndex++)
	{
		const auto &meshlet        = meshlets[meshletIndex];
		auto        getVertexIndex = [&](std::size_t index) {
            return primitive.meshletVertexIndices[primitive.meshletIndices[index + meshlet.triangle_offset] + meshlet.vertex_offset];
		};

		const std::size_t triangleCount = meshlet.triangle_count;
		// for each triangle of the meshlet
		for (std::size_t triangleIndex = 0; triangleIndex < triangleCount; triangleIndex++)
		{
			// for each edge of the triangle
			for (std::size_t i = 0; i < 3; i++)
			{
				MeshletEdge edge{getVertexIndex(i + triangleIndex * 3), getVertexIndex(((i + 1) % 3) + triangleIndex * 3)};
				edges2Meshlets[edge].push_back(meshletIndex);
				meshlets2Edges[meshletIndex].emplace_back(edge);
			}
		}
	}

	// remove edges which are not connected to 2 different meshlets
	std::erase_if(edges2Meshlets, [&](const auto &pair) {
		return pair.second.size() <= 1;
	});

	if (edges2Meshlets.empty())
	{
		return groupWithAllMeshlets();
	}

	// at this point, we have basically built a graph of meshlets, in which edges represent which meshlets are connected together

	// idx_t comes from METIS and corresponds to std::uint64_t in my build of METIS
	std::vector<MeshletGroup> groups;

	idx_t vertexCount = meshlets.size();            // vertex count, from the point of view of METIS, where Meshlet = vertex
	idx_t ncon        = 1;                          // only one constraint, minimum required by METIS
	idx_t nparts      = meshlets.size() / 8;        // groups of 4 // -> 8
	assert(nparts > 1);
	idx_t options[METIS_NOPTIONS];
	METIS_SetDefaultOptions(options);

	// edge-cut, ie minimum cost betweens groups.
	options[METIS_OPTION_OBJTYPE] = METIS_OBJTYPE_CUT;
	options[METIS_OPTION_CCORDER] = 1;        // identify connected components first

	// prepare storage for partition data
	// each vertex will get its partition index inside this vector after the edge-cut
	std::vector<idx_t> partition(vertexCount);

	// xadj
	std::vector<idx_t> xadjacency;
	xadjacency.reserve(vertexCount + 1);

	// adjncy
	std::vector<idx_t> edgeAdjacency;
	// weight of each edge
	std::vector<idx_t> edgeWeights;

	for (std::size_t meshletIndex = 0; meshletIndex < meshlets.size(); meshletIndex++)
	{
		std::size_t startIndexInEdgeAdjacency = edgeAdjacency.size();
		for (const auto &edge : meshlets2Edges[meshletIndex])
		{
			auto connectionsIter = edges2Meshlets.find(edge);
			if (connectionsIter == edges2Meshlets.end())
			{
				continue;
			}
			const auto &connections = connectionsIter->second;
			for (const auto &connectedMeshlet : connections)
			{
				if (connectedMeshlet != meshletIndex)
				{
					auto existingEdgeIter = std::find(edgeAdjacency.begin() + startIndexInEdgeAdjacency, edgeAdjacency.end(), connectedMeshlet);
					if (existingEdgeIter == edgeAdjacency.end())
					{
						// first time we see this connection to the other meshlet
						edgeAdjacency.emplace_back(connectedMeshlet);
						edgeWeights.emplace_back(1);
					}
					else
					{
						// not the first time! increase number of times we encountered this meshlet
						std::ptrdiff_t d = std::distance(edgeAdjacency.begin(), existingEdgeIter);
						assert(d >= 0);
						assert(d < edgeWeights.size());
						edgeWeights[d]++;
					}
				}
			}
		}
		xadjacency.push_back(startIndexInEdgeAdjacency);
	}
	xadjacency.push_back(edgeAdjacency.size());
	assert(xadjacency.size() == meshlets.size() + 1);
	assert(edgeAdjacency.size() == edgeWeights.size());
	
	for (const std::size_t &edgeAdjIndex : xadjacency)
	{
		assert(edgeAdjIndex <= edgeAdjacency.size());
	}
	for (const std::size_t &vertexIndex : edgeAdjacency)
	{
		assert(vertexIndex <= vertexCount);
	}

	idx_t edgeCut;        // final cost of the cut found by METIS
	int   result = METIS_PartGraphKway(&vertexCount,
	                                   &ncon,
	                                   xadjacency.data(),
	                                   edgeAdjacency.data(),
	                                   NULL,           /* vertex weights */
	                                   NULL,           /* vertex size */
	                                   edgeWeights.data(),
	                                   &nparts,
	                                   NULL,
	                                   NULL,
	                                   options,
	                                   &edgeCut,
	                                   partition.data());

	assert(result == METIS_OK);

	// ===== Group meshlets together
	groups.resize(nparts);
	for (std::size_t i = 0; i < meshlets.size(); i++)
	{
		idx_t partitionNumber = partition[i];
		groups[partitionNumber].meshlets.push_back(i);
	}
	//for (int i=0;i<nparts;i++)
	//	if (groups[i].meshlets.size() == 0)
	//	{
	//		int j = 0;
	//	}
	return groups;
	// end of function
}


// 从最新一级别的LOD重新拆分meshlet，附加到原来的primitive数组后面
static void appendMeshlets(MeshPrimitiveData &primitive, std::span<std::uint32_t> indexBuffer)
{
	constexpr std::size_t maxVertices  = 64;
	constexpr std::size_t maxTriangles = 124;
	const float           coneWeight   = 0.0f;        // for occlusion culling, currently unused

	const std::size_t            meshletOffset = primitive.meshlets.size();
	const std::size_t            vertexOffset  = primitive.meshletVertexIndices.size();
	const std::size_t            indexOffset   = primitive.meshletIndices.size();
	const std::size_t            maxMeshlets   = meshopt_buildMeshletsBound(indexBuffer.size(), maxVertices, maxTriangles);
	
	std::vector<meshopt_Meshlet> meshoptMeshlets(maxMeshlets);
	std::vector<unsigned int>  meshletVertexIndices(maxMeshlets * maxVertices);
	std::vector<unsigned char> meshletTriangles(maxMeshlets * maxTriangles * 3);

	auto vertex_positions = reinterpret_cast<const float *>(primitive.attributes.at("position").data.data());

	const std::size_t      meshletCount = meshopt_buildMeshlets(meshoptMeshlets.data(), meshletVertexIndices.data(), meshletTriangles.data(),        // meshlet outputs
	                                                            indexBuffer.data(), indexBuffer.size(),                                              // original index buffer
	                                                            vertex_positions,                                                                   // pointer to position data
	                                                            primitive.vertex_count,                                                             // vertex count of original mesh
	                                                            sizeof(float) * 3,
	                                                            maxVertices, maxTriangles, coneWeight);
	const meshopt_Meshlet &last         = meshoptMeshlets[meshletCount - 1];
	const std::size_t      vertexCount  = last.vertex_offset + last.vertex_count;
	const std::size_t      indexCount   = last.triangle_offset + last.triangle_count * 3;
	primitive.meshletVertexIndices.resize(vertexOffset + vertexCount);
	primitive.meshletIndices.resize(indexOffset + indexCount);
	primitive.meshlets.resize(meshletOffset + meshletCount);        // remove over-allocated meshlets

    for (std::size_t index = 0; index < vertexCount; ++index) {
        primitive.meshletVertexIndices[vertexOffset + index] = meshletVertexIndices[index];
    }

    for (std::size_t index = 0; index < indexCount; ++index) {
        primitive.meshletIndices[indexOffset + index] = meshletTriangles[index];
    }

    for (std::size_t index = 0; index < meshletCount; ++index) {
        auto &meshoptMeshlet = meshoptMeshlets[index];
        auto &meshlet  = primitive.meshlets[meshletOffset + index];

        meshlet.vertex_offset      = vertexOffset + meshoptMeshlet.vertex_offset;
		meshlet.vertex_count	   = meshoptMeshlet.vertex_count;

        meshlet.triangle_offset    = indexOffset + meshoptMeshlet.triangle_offset;
		meshlet.triangle_count	   = meshoptMeshlet.triangle_count;

		meshopt_Bounds meshlet_bounds = meshopt_computeMeshletBounds(
		    meshletVertexIndices.data() + meshoptMeshlet.vertex_offset,
		    meshletTriangles.data() + meshoptMeshlet.triangle_offset,
		    meshoptMeshlet.triangle_count, vertex_positions, primitive.vertex_count, sizeof(float) * 3);

		meshlet.center = glm::vec3(meshlet_bounds.center[0], meshlet_bounds.center[1], meshlet_bounds.center[2]);
		meshlet.radius = meshlet_bounds.radius;

		meshlet.cone_axis   = glm::vec3(meshlet_bounds.cone_axis[0], meshlet_bounds.cone_axis[1], meshlet_bounds.cone_axis[2]);
		meshlet.cone_cutoff = meshlet_bounds.cone_cutoff;

		meshlet.cone_apex = glm::vec3(meshlet_bounds.cone_apex[0], meshlet_bounds.cone_apex[1], meshlet_bounds.cone_apex[2]);

    }
}

void generateClusterHierarchy(MeshPrimitiveData &primitive)
{
	// level 0
	// tell meshoptimizer to generate meshlets
	std::vector<uint32_t> index_data_32;
	if (primitive.index_type == vk::IndexType::eUint16)
	{
		const uint16_t *index_data_16 = reinterpret_cast<const uint16_t *>(primitive.indices.data());
		index_data_32.resize(primitive.index_count);
		for (size_t i = 0; i < primitive.index_count; ++i)
		{
			index_data_32[i] = static_cast<uint32_t>(index_data_16[i]);
		}
	}
	else if (primitive.index_type == vk::IndexType::eUint32)
	{
		index_data_32.assign(
		    reinterpret_cast<const uint32_t *>(primitive.indices.data()),
		    reinterpret_cast<const uint32_t *>(primitive.indices.data()) + primitive.index_count);
	}
	//auto       &indexBuffer           = primitive.indices;
	std::size_t previousMeshletsStart = 0;
	appendMeshlets(primitive, index_data_32);
	// level n+1
	const int maxLOD = 5;        // I put a hard limit, but 25 might already be too high for some models
	for (int lod = 0; lod < maxLOD; ++lod)
	{
		float tLod = lod / (float) maxLOD;

		// find out the meshlets of the LOD n
		std::span<Meshlet> previousLevelMeshlets = std::span{primitive.meshlets.data() + previousMeshletsStart, primitive.meshlets.size() - previousMeshletsStart};
		if (previousLevelMeshlets.size() <= 1)
		{
			return;        // we have reached the end
		}

		std::vector<MeshletGroup> groups = groupMeshlets(primitive, previousLevelMeshlets);
		
		// ===== Simplify groups
		const std::size_t newMeshletStart = primitive.meshlets.size();
		for (const auto &group : groups)
		{
			// meshlets vector is modified during the loop
			previousLevelMeshlets = std::span{primitive.meshlets.data() + previousMeshletsStart, primitive.meshlets.size() - previousMeshletsStart};
			std::vector<uint32_t> groupVertexIndices;

			// add cluster vertices to this group
			for (const auto &meshletIndex : group.meshlets)
			{
				const auto &meshlet = previousLevelMeshlets[meshletIndex];
				std::size_t start   = groupVertexIndices.size();
				groupVertexIndices.resize(start + meshlet.triangle_count * 3);
				for (std::size_t j = 0; j < meshlet.triangle_count * 3; j++)
				{
					groupVertexIndices[j + start] = primitive.meshletVertexIndices[primitive.meshletIndices[meshlet.triangle_offset + j] + meshlet.vertex_offset];
				}
			}

			// simplify this group
			const float  threshold        = 0.5f;
			std::size_t  targetIndexCount = groupVertexIndices.size() * threshold;
			//if (targetIndexCount == 0)
			//{
			//	int i = 1;
			//}
			float        targetError      = 0.9f * tLod + 0.01f * (1 - tLod);
			unsigned int options     = meshopt_SimplifyErrorAbsolute;
			//unsigned int options     = meshopt_SimplifyLockBorder;        // we want all group borders to be locked (because they are shared between groups)

			std::vector<uint32_t> simplifiedIndexBuffer;
			simplifiedIndexBuffer.resize(groupVertexIndices.size());
			float simplificationError = 0.f;

			auto vertex_positions = reinterpret_cast<const float *>(primitive.attributes.at("position").data.data());

			std::size_t simplifiedIndexCount = meshopt_simplify(simplifiedIndexBuffer.data(),                                                           // output
			                                                    groupVertexIndices.data(), groupVertexIndices.size(),                                   // index buffer
			                                                    vertex_positions,                                            // pointer to position data
			                                                    primitive.vertex_count,                                      // vertex count of original mesh
			                                                    sizeof(float) * 3,
			                                                    targetIndexCount, targetError, options, &simplificationError);
			simplifiedIndexBuffer.resize(simplifiedIndexCount);
			// ===== Generate meshlets for this group
			appendMeshlets(primitive, simplifiedIndexBuffer);
			for (std::size_t i = newMeshletStart; i < primitive.meshlets.size(); i++)
			{
				primitive.meshlets[i].lod = lod + 1;
			}
		}
		previousMeshletsStart = newMeshletStart;
	}
}
}        // namespace xihe::sg
