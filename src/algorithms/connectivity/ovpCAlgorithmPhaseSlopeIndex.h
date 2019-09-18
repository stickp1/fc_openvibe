#if defined(TARGET_HAS_ThirdPartyEIGEN)

#ifndef __OpenViBEPlugins_Algorithm_PhaseSlopeIndex_H__
#define __OpenViBEPlugins_Algorithm_PhaseSlopeIndex_H__

#include "../../ovp_defines.h"
#include <openvibe/ov_all.h>
#include <toolkit/ovtk_all.h>
#include "ovpCConnectivityAlgorithm.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include "../basic/ovpCWindowFunctions.h"
#include <iostream>

namespace OpenViBEPlugins
{
	namespace SignalProcessing
	{
		class CAlgorithmPhaseSlopeIndex : public OpenViBEPlugins::CConnectivityAlgorithm
		{
		public:

			virtual void release(void) { delete this; }

			bool computePeriodogram(const Eigen::VectorXd& vecXcdInput, Eigen::MatrixXcd& matXcdPeriodograms,const Eigen::VectorXd& vecXdWindow, const uint32_t& ui32NSegments, const uint32_t& ui32LSegments, const uint32_t& ui32NOverlap);
			bool powerSpectralDensity(const Eigen::VectorXd& vecXdInput, Eigen::VectorXd& vecXdOutput, const Eigen::VectorXd& vecXdWindow, const uint32_t& ui32NSegments, const uint32_t& ui32LSegments, const uint32_t& ui32NOverlap);
			bool crossSpectralDensity(const Eigen::VectorXd& vecXdInput1, const Eigen::VectorXd& vecXdInput2, Eigen::VectorXcd& vecXcdOutput, const Eigen::VectorXd& vecXdWindow, const uint32_t& ui32NSegments, const uint32_t& ui32LSegments, const uint32_t& ui32NOverlap);

			virtual bool initialize(void);
			virtual bool uninitialize(void);
			virtual bool process(void);


			_IsDerivedFromClass_Final_(OpenViBEPlugins::CConnectivityAlgorithm, OVP_TypeId_Algorithm_PhaseSlopeIndex);

		protected:

			OpenViBE::Kernel::TParameterHandler <OpenViBE::IMatrix*> op_pMatrixMean;
			OpenViBE::Kernel::TParameterHandler <OpenViBE::IMatrix*> op_pMatrixSpectrum;
			OpenViBE::Kernel::TParameterHandler <uint64_t > ip_ui64SegmentLength;
			OpenViBE::Kernel::TParameterHandler <uint64_t > ip_ui64Overlap;
			OpenViBE::Kernel::TParameterHandler <uint64_t > ip_ui64WindowType;
			OpenViBE::Kernel::TParameterHandler <OpenViBE::IMatrix*> op_FrequencyAbscissaVector;
			OpenViBE::Kernel::TParameterHandler <uint64_t> ip_ui64FreqWindow;
		

		private:
			// why members?
			Eigen::VectorXd m_vecXdPowerSpectrum1;
			Eigen::VectorXd m_vecXdPowerSpectrum2;
			Eigen::VectorXcd m_vecXcdCrossSpectrum;

			Eigen::VectorXd m_vecXdWindow;  // Window weight vector
			OpenViBE::float64 m_f64U;       // Window's normalization constant

			WindowFunctions m_oWindow;

			Eigen::FFT< double, Eigen::internal::kissfft_impl<double > > m_oFFT; // Instance of the fft transform


		};

		class CAlgorithmPhaseSlopeIndexDesc : public OpenViBEPlugins::CConnectivityAlgorithmDesc
		{
		public:
			virtual void release(void) { }

			virtual OpenViBE::CString getName(void) const                { return OpenViBE::CString("Phase Slope Index"); }
			virtual OpenViBE::CString getAuthorName(void) const          { return OpenViBE::CString("Cristiano Berhanu"); }
			virtual OpenViBE::CString getAuthorCompanyName(void) const   { return OpenViBE::CString("LaSEEB, Instituto Superior Técnico, University of Lisbon"); }
			virtual OpenViBE::CString getShortDescription(void) const    { return OpenViBE::CString("Compute the Phase Slope Index"); }
			virtual OpenViBE::CString getDetailedDescription(void) const { return OpenViBE::CString("As described in Nolte et al (2008)"); }
			virtual OpenViBE::CString getCategory(void) const            { return OpenViBE::CString("Signal processing/Connectivity"); }
			virtual OpenViBE::CString getVersion(void) const             { return OpenViBE::CString("0.1"); }
			virtual OpenViBE::CString getStockItemName(void) const       { return OpenViBE::CString("gtk-execute"); }

			virtual OpenViBE::CIdentifier getCreatedClass(void) const    { return OVP_TypeId_Algorithm_PhaseSlopeIndex; }
			virtual OpenViBE::Plugins::IPluginObject* create(void)       { return new OpenViBEPlugins::SignalProcessing::CAlgorithmPhaseSlopeIndex; }

			virtual OpenViBE::boolean getAlgorithmPrototype(
					OpenViBE::Kernel::IAlgorithmProto& rAlgorithmPrototype) const
			{


				//CConnectivityAlgorithmDesc::getAlgorithmPrototype(rAlgorithmPrototype);
				
				rAlgorithmPrototype.addInputParameter(OVP_Algorithm_Connectivity_InputParameterId_InputMatrix1, "Signal 1", OpenViBE::Kernel::ParameterType_Matrix);
				rAlgorithmPrototype.addInputParameter(OVP_Algorithm_Connectivity_InputParameterId_InputMatrix2, "Signal 2", OpenViBE::Kernel::ParameterType_Matrix);
				rAlgorithmPrototype.addInputParameter(OVP_Algorithm_Connectivity_InputParameterId_LookupMatrix, "Pairs of channels", OpenViBE::Kernel::ParameterType_Matrix);
				rAlgorithmPrototype.addInputParameter(OVP_Algorithm_Connectivity_InputParameterId_ui64SamplingRate1, "Sampling Rate of signal 1", OpenViBE::Kernel::ParameterType_UInteger);

				rAlgorithmPrototype.addInputParameter(OVP_Algorithm_PhaseSlopeIndex_InputParameterId_Window, "Window method", OpenViBE::Kernel::ParameterType_Enumeration, OVP_TypeId_WindowType);
				rAlgorithmPrototype.addInputParameter(OVP_Algorithm_PhaseSlopeIndex_InputParameterId_SegLength, "Length of segments", OpenViBE::Kernel::ParameterType_Integer, 32);
				rAlgorithmPrototype.addInputParameter(OVP_Algorithm_PhaseSlopeIndex_InputParameterId_Overlap, "Overlap (percentage)", OpenViBE::Kernel::ParameterType_Integer);
				rAlgorithmPrototype.addInputParameter(OVP_Algorithm_PhaseSlopeIndex_InputParameterId_FreqWindow, "Frequency window", OpenViBE::Kernel::ParameterType_Integer);

				rAlgorithmPrototype.addOutputParameter(OVP_Algorithm_Connectivity_OutputParameterId_OutputMatrix,    "Mean PSI signal", OpenViBE::Kernel::ParameterType_Matrix);
				rAlgorithmPrototype.addOutputParameter(OVP_Algorithm_PhaseSlopeIndex_OutputParameterId_OutputMatrixSpectrum, "PSI spectrum", OpenViBE::Kernel::ParameterType_Matrix);
				rAlgorithmPrototype.addOutputParameter(OVP_Algorithm_PhaseSlopeIndex_OutputParameterId_FreqVector, "Frequency Vector", OpenViBE::Kernel::ParameterType_Matrix);
				
				rAlgorithmPrototype.addInputTrigger   (OVP_Algorithm_Connectivity_InputTriggerId_Initialize,   "Initialize");
				rAlgorithmPrototype.addInputTrigger   (OVP_Algorithm_Connectivity_InputTriggerId_Process,      "Process");
				rAlgorithmPrototype.addOutputTrigger  (OVP_Algorithm_Connectivity_OutputTriggerId_ProcessDone, "Process done");

				return true;
			}

			_IsDerivedFromClass_Final_(OpenViBEPlugins::CConnectivityAlgorithmDesc, OVP_TypeId_Algorithm_PhaseSlopeIndexDesc);
		};
	};  // namespace SignalProcessing
}; // namespace OpenViBEPlugins

#endif //__OpenViBEPlugins_Algorithm_PhaseSlopeIndex_H__
#endif //TARGET_HAS_ThirdPartyEIGEN
