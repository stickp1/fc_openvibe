#ifndef __OpenViBEPlugins_BoxAlgorithm_Null_H__
#define __OpenViBEPlugins_BoxAlgorithm_Null_H__

#include <toolkit/ovtk_all.h>

#define OVP_ClassId_BoxAlgorithm_Null                                              OpenViBE::CIdentifier(0x601118A8, 0x14BF700F)
#define OVP_ClassId_BoxAlgorithm_NullDesc                                          OpenViBE::CIdentifier(0x6BD21A21, 0x0A5E685A)

namespace OpenViBEPlugins
{
	namespace SignalProcessing
	{
		class CBoxAlgorithmNull : public OpenViBEToolkit::TBoxAlgorithm<OpenViBE::Plugins::IBoxAlgorithm>
		{
		public:

			virtual void release(void);
			virtual OpenViBE::boolean processInput(OpenViBE::uint32 ui32InputIndex);
			virtual OpenViBE::boolean process(void);

			_IsDerivedFromClass_Final_(OpenViBE::Plugins::IBoxAlgorithm, OVP_ClassId_BoxAlgorithm_Null)
		};

		class CBoxAlgorithmNullDesc : public OpenViBE::Plugins::IBoxAlgorithmDesc
		{
		public:

			virtual void release(void) { }
			virtual OpenViBE::CString getName(void) const                { return OpenViBE::CString("Null"); }
			virtual OpenViBE::CString getAuthorName(void) const          { return OpenViBE::CString("Jussi T. Lindgren"); }
			virtual OpenViBE::CString getAuthorCompanyName(void) const   { return OpenViBE::CString("Inria"); }
			virtual OpenViBE::CString getShortDescription(void) const    { return OpenViBE::CString("Consumes input and produces nothing. It can be used to show scenario design intent."); }
			virtual OpenViBE::CString getDetailedDescription(void) const { return OpenViBE::CString("Directing to Null instead of leaving a box output unconnected may add a tiny overhead."); }
			virtual OpenViBE::CString getCategory(void) const            { return OpenViBE::CString("Signal processing/Basic"); }
			virtual OpenViBE::CString getVersion(void) const             { return OpenViBE::CString("1.0"); }
			virtual OpenViBE::CString getSoftwareComponent(void) const   { return OpenViBE::CString("openvibe-extras"); }
			virtual OpenViBE::CString getAddedSoftwareVersion(void) const   { return OpenViBE::CString("0.0.0"); }
			virtual OpenViBE::CString getUpdatedSoftwareVersion(void) const { return OpenViBE::CString("0.0.0"); }
			
			virtual OpenViBE::CIdentifier getCreatedClass(void) const    { return OVP_ClassId_BoxAlgorithm_Null; }
			virtual OpenViBE::Plugins::IPluginObject* create(void)       { return new OpenViBEPlugins::SignalProcessing::CBoxAlgorithmNull(); }

			virtual OpenViBE::boolean getBoxPrototype(
				OpenViBE::Kernel::IBoxProto& rPrototype) const
			{
				rPrototype.addInput ("Input stream",  OV_TypeId_EBMLStream);
				rPrototype.addFlag  (OpenViBE::Kernel::BoxFlag_CanAddInput);
				rPrototype.addFlag  (OpenViBE::Kernel::BoxFlag_CanModifyInput);
				return true;
			}

			_IsDerivedFromClass_Final_(OpenViBE::Plugins::IBoxAlgorithmDesc, OVP_ClassId_BoxAlgorithm_NullDesc)
		};
	};
};

#endif // __OpenViBEPlugins_BoxAlgorithm_Null_H__
