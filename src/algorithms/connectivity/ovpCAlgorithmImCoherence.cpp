#if defined(TARGET_HAS_ThirdPartyEIGEN)

#include "ovpCAlgorithmImCoherence.h"
#include <cmath>
#include <complex>
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>


using namespace OpenViBE;
using namespace OpenViBE::Kernel;
using namespace OpenViBE::Plugins;

using namespace OpenViBEPlugins;
using namespace OpenViBEPlugins::SignalProcessing;

using namespace Eigen;
using namespace std;

// Let s=w.*x(k) be the windowed input signal x at segment k. The column k of the output matrix will be fft(s).
bool CAlgorithmImCoherence::computePeriodogram(const VectorXd& vecXdInput, Eigen::MatrixXcd& matXcdSignalFourier, const Eigen::VectorXd& vecXdWindow, const uint32_t& ui32NSegments, const uint32_t& ui32LSegments, const uint32_t& ui32NOverlap)
{
	matXcdSignalFourier = MatrixXcd::Zero(ui32LSegments, ui32NSegments);

	// Segment input vector and apply window to segment each segment
	for(uint32_t k = 0; k<ui32NSegments; k++)
	{
		VectorXd l_vecXdWindowedSegments(ui32LSegments);

		for(uint32_t i = 0; i<ui32LSegments; i++)
		{
			l_vecXdWindowedSegments(i) = vecXdInput(i+k*(ui32LSegments-ui32NOverlap))*vecXdWindow(i); 
		}

		// Remove the DC component of each channel. Without this we get really high coherences spread all over the 
		// spectrum when using e.g. the motor imagery dataset bundled with OpenViBE. @todo before or after windowing? here after.
		// l_vecXdWindowedSegments = l_vecXdWindowedSegments - VectorXd::Ones(ui32LSegments) * l_vecXdWindowedSegments.mean();

		VectorXcd l_vecXcdFourier = VectorXcd::Zero(ui32LSegments);

		// FFT of windowed segments
		m_oFFT.fwd(l_vecXcdFourier,l_vecXdWindowedSegments);

		matXcdSignalFourier.col(k) = l_vecXcdFourier;

	}
	return true;
}


bool CAlgorithmImCoherence::powerSpectralDensity(const VectorXd& vecXdInput, VectorXd& vecXdOutput, const VectorXd& vecXdWindow, const uint32_t& ui32NSegments, const uint32_t& ui32LSegments, const uint32_t& ui32NOverlap)
{
	// Compute periodograms
	MatrixXcd l_matXcdSignalFourier;

	CAlgorithmImCoherence::computePeriodogram(vecXdInput, l_matXcdSignalFourier, vecXdWindow, ui32NSegments, ui32LSegments, ui32NOverlap);
	//cout << "sf " << l_matXcdSignalFourier << "\n";

	// vecXdOutput(i) will be the power for the band i across segments (time) as summed from the periodogram
	vecXdOutput = VectorXd::Zero(ui32LSegments);
	for(uint32_t k = 0; k<ui32NSegments; k++)
	{
		vecXdOutput += (l_matXcdSignalFourier.col(k).cwiseProduct(l_matXcdSignalFourier.col(k).conjugate())).real()/m_f64U;
	}
	vecXdOutput /= ui32NSegments;

 //   cout << "vxO " << vecXdOutput.transpose() << "\n";

	return true;

}


bool CAlgorithmImCoherence::crossSpectralDensity(const VectorXd& vecXdInput1, const VectorXd& vecXdInput2, VectorXcd& vecXcdOutput, const VectorXd& vecXdWindow, const uint32_t& ui32NSegments, const uint32_t& ui32LSegments, const uint32_t& ui32NOverlap)
{
	MatrixXcd l_matXcdSignalFourier1;
	MatrixXcd l_matXcdSignalFourier2;

	//Compute periodograms for input 1 and 2
	CAlgorithmImCoherence::computePeriodogram(vecXdInput1, l_matXcdSignalFourier1, vecXdWindow, ui32NSegments, ui32LSegments, ui32NOverlap);
	CAlgorithmImCoherence::computePeriodogram(vecXdInput2, l_matXcdSignalFourier2, vecXdWindow, ui32NSegments, ui32LSegments, ui32NOverlap);

	vecXcdOutput = VectorXcd::Zero(ui32LSegments);

	for(uint32_t k = 0; k<ui32NSegments; k++)
	{
		vecXcdOutput += l_matXcdSignalFourier1.col(k).cwiseProduct(l_matXcdSignalFourier2.col(k).conjugate())/m_f64U;
	}

	vecXcdOutput = vecXcdOutput / ui32NSegments;

	// cout << "vxX " << vecXcdOutput.transpose() << "\n";

	return true;

}

bool CAlgorithmImCoherence::initialize(void)
{

	ip_pSignal1.initialize(this->getInputParameter(OVP_Algorithm_Connectivity_InputParameterId_InputMatrix1));
	ip_pSignal2.initialize(this->getInputParameter(OVP_Algorithm_Connectivity_InputParameterId_InputMatrix2));
	ip_ui64SamplingRate1.initialize(this->getInputParameter(OVP_Algorithm_Connectivity_InputParameterId_ui64SamplingRate1));

	ip_pChannelPairs.initialize(this->getInputParameter(OVP_Algorithm_Connectivity_InputParameterId_LookupMatrix));
	op_pMatrixMean.initialize(this->getOutputParameter(OVP_Algorithm_Connectivity_OutputParameterId_OutputMatrix));
	op_pMatrixSpectrum.initialize(this->getOutputParameter(OVP_Algorithm_ImCoherence_OutputParameterId_OutputMatrixSpectrum));
	op_FrequencyAbscissaVector.initialize(this->getOutputParameter(OVP_Algorithm_ImCoherence_OutputParameterId_FreqVector));

	ip_ui64WindowType.initialize(this->getInputParameter(OVP_Algorithm_ImCoherence_InputParameterId_Window));
	ip_ui64SegmentLength.initialize(this->getInputParameter(OVP_Algorithm_ImCoherence_InputParameterId_SegLength));
	ip_ui64Overlap.initialize(this->getInputParameter(OVP_Algorithm_ImCoherence_InputParameterId_Overlap));

	// Set default values
	ip_ui64WindowType = OVP_TypeId_WindowType_Welch.toUInteger();
	ip_ui64SegmentLength = 32;
	ip_ui64Overlap = 50;

	return true;
}

bool CAlgorithmImCoherence::uninitialize(void)
{

	ip_pSignal1.uninitialize();
	ip_pSignal2.uninitialize();
	ip_ui64SamplingRate1.uninitialize();

	ip_pChannelPairs.uninitialize();
	op_pMatrix.uninitialize();
	op_pMatrixSpectrum.uninitialize();
	op_FrequencyAbscissaVector.uninitialize();

	ip_ui64WindowType.uninitialize();
	ip_ui64SegmentLength.uninitialize();
	ip_ui64Overlap.uninitialize();

	return true;
}


bool CAlgorithmImCoherence::process(void)
{
	// Inputs
	const IMatrix* l_pInputMatrix1 = ip_pSignal1;
	const IMatrix* l_pInputMatrix2 = ip_pSignal2;
	const IMatrix* l_pChannelPairs = ip_pChannelPairs;

	const uint64_t l_ui64SamplingRate = ip_ui64SamplingRate1;
	const uint32_t l_ui32SamplesPerChannel1 = l_pInputMatrix1->getDimensionSize(1);
	const uint32_t l_ui32SamplesPerChannel2 = l_pInputMatrix2->getDimensionSize(1);
	const uint32_t l_ui32PairsCount = ip_pChannelPairs->getDimensionSize(0)/2;

	const float64* l_pMatrixBuffer1 = l_pInputMatrix1->getBuffer();
	const float64* l_pMatrixBuffer2 = l_pInputMatrix2->getBuffer();
	const float64* l_pChannelBuffer = l_pChannelPairs->getBuffer();

	const uint32_t l_ui32SegmentsLength = (uint32_t) ip_ui64SegmentLength;
	const uint64_t  l_ui64WindowType = ip_ui64WindowType;
	const uint32_t l_ui32OverlapPercent = (uint32_t) ip_ui64Overlap;

	// Outputs
	IMatrix* l_pOutputMatrixMeanCoherence = op_pMatrixMean;
	IMatrix* l_pOutputMatrixCoherenceSpectrum = op_pMatrixSpectrum;

	if(this->isInputTriggerActive(OVP_Algorithm_Connectivity_InputTriggerId_Initialize))
	{

		OV_ERROR_UNLESS_KRF(
			l_ui32SamplesPerChannel1 == l_ui32SamplesPerChannel2,
			"Can't compute MSCoherence on two signals with different lengths",
			OpenViBE::Kernel::ErrorType::BadProcessing);

		OV_ERROR_UNLESS_KRF(
			l_ui32SamplesPerChannel1 !=0 && l_ui32SamplesPerChannel2 != 0,
			"Can't compute MSCoherence, input signal size = 0",
			OpenViBE::Kernel::ErrorType::BadProcessing);
		
		OV_ERROR_UNLESS_KRF(
			l_ui32OverlapPercent >= 0 && l_ui32OverlapPercent <= 100,
			"Overlap must be a value between 0 and 100",
			OpenViBE::Kernel::ErrorType::BadProcessing);

		OV_ERROR_UNLESS_KRF(
			l_ui32SegmentsLength > 0,
			"Segments must have a strictly positive length (>0)",
			OpenViBE::Kernel::ErrorType::BadProcessing);

		// Setting window vector. We do this only once after we know the real segment length.
		m_vecXdWindow = VectorXd::Zero(l_ui32SegmentsLength);

		// Getting window vector
		if (l_ui64WindowType == OVP_TypeId_WindowType_Bartlett)
		{
			m_oWindow.bartlett(m_vecXdWindow, l_ui32SegmentsLength);
		}
		else if (l_ui64WindowType == OVP_TypeId_WindowType_Hamming)
		{
			m_oWindow.hamming(m_vecXdWindow, l_ui32SegmentsLength);
		}
		else if (l_ui64WindowType == OVP_TypeId_WindowType_Hann)
		{
			m_oWindow.hann(m_vecXdWindow, l_ui32SegmentsLength);
		}
		else if (l_ui64WindowType == OVP_TypeId_WindowType_Parzen)
		{
			m_oWindow.parzen(m_vecXdWindow, l_ui32SegmentsLength);
		}
		else if (l_ui64WindowType == OVP_TypeId_WindowType_Welch)
		{
			m_oWindow.welch(m_vecXdWindow, l_ui32SegmentsLength);
		}
		else // else rectangular window
		{
			m_vecXdWindow = VectorXd::Ones(l_ui32SegmentsLength);
		}
		// cout<<"window = "<<m_vecXdWindow.transpose()<<endl;

		// Calculate window normalization constant m_f64U. A 1/ui32LSegments factor has been removed since it will be canceled
		m_f64U = 0;
		for(uint32_t i = 0; i<l_ui32SegmentsLength; i++)
		{
			m_f64U += pow(m_vecXdWindow(i),2);
		}

		// Setting size of outputs
		l_pOutputMatrixMeanCoherence->setDimensionCount(2); // the output matrix will have 2 dimensions
		l_pOutputMatrixMeanCoherence->setDimensionSize(0,l_ui32PairsCount);
		l_pOutputMatrixMeanCoherence->setDimensionSize(1,1); // Compute the mean so only one value

		l_pOutputMatrixCoherenceSpectrum->setDimensionCount(2); // the output matrix will have 2 dimensions
		l_pOutputMatrixCoherenceSpectrum->setDimensionSize(0,l_ui32PairsCount);
		l_pOutputMatrixCoherenceSpectrum->setDimensionSize(1, l_ui32SegmentsLength);

		op_FrequencyAbscissaVector->setDimensionCount(1);
		op_FrequencyAbscissaVector->setDimensionSize(0, l_ui32SegmentsLength);

		// Setting name of output channels for visualization
		for(uint32_t i=0;i<l_ui32PairsCount;i++)
		{
			const uint32_t l_ui32Index=2*i;
			CString l_name1 = l_pInputMatrix1->getDimensionLabel(0, static_cast<uint32_t>(l_pChannelBuffer[l_ui32Index+0]));
			CString l_name2 = l_pInputMatrix2->getDimensionLabel(0, static_cast<uint32_t>(l_pChannelBuffer[l_ui32Index+1]));
			CString l_name = l_name1+" "+l_name2;

			l_pOutputMatrixMeanCoherence->setDimensionLabel(0,i,l_name);
			l_pOutputMatrixCoherenceSpectrum->setDimensionLabel(0,i,l_name);
		}

		// Create frequency vector for the spectrum encoder
		float64* l_pFrequencyVectorBuffer = op_FrequencyAbscissaVector->getBuffer();
		for (uint32_t i = 0; i < l_ui32SegmentsLength / 2; i++) // this was changed from the original - we are repeating values in spectrum, so values must be repeated in frequencies as well
		{
			l_pFrequencyVectorBuffer[2 * i] = i * (static_cast<double>(l_ui64SamplingRate) / l_ui32SegmentsLength);
			l_pFrequencyVectorBuffer[2 * i + 1] = i * (static_cast<double>(l_ui64SamplingRate) / l_ui32SegmentsLength);
			cout << op_FrequencyAbscissaVector->getBuffer()[2 * i] << endl;
			cout << op_FrequencyAbscissaVector->getBuffer()[2 * i + 1] << endl;
		}
	}

	if(this->isInputTriggerActive(OVP_Algorithm_Connectivity_InputTriggerId_Process))
	{
		const uint32_t l_ui32NOverlap = l_ui32OverlapPercent*l_ui32SegmentsLength/100; // Convert percentage in samples

		uint32_t l_ui32NbSegments = 0;
		float64 l_f64MeanCohere;

		// Calculate number of segment on data set giving segment's length and overlap
		if(l_ui32NOverlap != 0)
		{
			l_ui32NbSegments = (l_ui32SamplesPerChannel1 - l_ui32SegmentsLength)/l_ui32NOverlap + 1;
		}
		else
		{
			l_ui32NbSegments = l_ui32SamplesPerChannel1/l_ui32SegmentsLength;
		}


		//_______________________________________________________________________________________
		//
		// Compute Imcoherency for each pair
		//_______________________________________________________________________________________
		//

		for(uint32_t channel = 0; channel < l_ui32PairsCount; channel++)
		{
			VectorXd l_vecXdChannelToCompare1(l_ui32SamplesPerChannel1);
			VectorXd l_vecXdChannelToCompare2(l_ui32SamplesPerChannel2);
			l_f64MeanCohere = 0;
		
			//_______________________________________________________________________________________
			//
			// Form pairs with the lookup matrix given
			//_______________________________________________________________________________________
			//

			const uint32_t l_ui32ChannelIndex = 2*channel; //Index of a single channel
			const uint32_t l_ui32Channel1 = (uint32_t)l_pChannelBuffer[l_ui32ChannelIndex+0];
			const uint32_t l_ui32Channel2 = (uint32_t)l_pChannelBuffer[l_ui32ChannelIndex+1];

			for(uint32_t sample = 0; sample < l_ui32SamplesPerChannel1; sample++)
			{
				l_vecXdChannelToCompare1(sample) = l_pMatrixBuffer1[sample+l_ui32Channel1*l_ui32SamplesPerChannel1];
			}
			for(uint32_t sample = 0; sample < l_ui32SamplesPerChannel2; sample++)
			{
				l_vecXdChannelToCompare2(sample) = l_pMatrixBuffer2[sample+l_ui32Channel2*l_ui32SamplesPerChannel2];
			}

			// Remove the DC component of each channel. Without this we get really high coherences spread all over the 
			// spectrum when using e.g. the motor imagery dataset bundled with OpenViBE. @todo before or after windowing? here before.
			l_vecXdChannelToCompare1 = l_vecXdChannelToCompare1 - VectorXd::Ones(l_ui32SamplesPerChannel1) * l_vecXdChannelToCompare1.mean();
			l_vecXdChannelToCompare2 = l_vecXdChannelToCompare2 - VectorXd::Ones(l_ui32SamplesPerChannel2) * l_vecXdChannelToCompare2.mean();

			/* Compute ImCoherency */
 			CAlgorithmImCoherence::powerSpectralDensity(l_vecXdChannelToCompare1, m_vecXdPowerSpectrum1, m_vecXdWindow, l_ui32NbSegments, l_ui32SegmentsLength, l_ui32NOverlap);
  			CAlgorithmImCoherence::powerSpectralDensity(l_vecXdChannelToCompare2, m_vecXdPowerSpectrum2, m_vecXdWindow, l_ui32NbSegments, l_ui32SegmentsLength, l_ui32NOverlap);
			CAlgorithmImCoherence::crossSpectralDensity(l_vecXdChannelToCompare1, l_vecXdChannelToCompare2, m_vecXcdCrossSpectrum, m_vecXdWindow, l_ui32NbSegments, l_ui32SegmentsLength, l_ui32NOverlap);

			const float64 l_f64Epsilon = 10e-3;	// discard bands with autospectral power smaller than this

			const VectorXcd l_vecXdCoherencyNum = m_vecXcdCrossSpectrum;
			const VectorXd  l_vecXdCoherencyDen = (m_vecXdPowerSpectrum1.cwiseProduct(m_vecXdPowerSpectrum2)).cwiseSqrt();
			const VectorXcd l_vecXcdCoherency = l_vecXdCoherencyNum.cwiseQuotient(l_vecXdCoherencyDen);
			
			// approach with variance stabilization
			//const VectorXd ones = VectorXd::Ones(l_ui32SegmentsLength); 
			//const VectorXd arctanh = (ones+l_vecXcdCoherency.cwiseAbs()).cwiseQuotient(ones-l_vecXcdCoherency.cwiseAbs()).array().log().matrix();
			//const VectorXcd l_vecXcdCoherence = l_vecXcdCoherency.cwiseQuotient(l_vecXcdCoherency.cwiseAbs())*0.5*arctanh;
			//const VectorXd l_vecXdCoherence = l_vecXcdCoherence.imag().cwiseAbs();
			
			// apprach without variance stabilization
			const VectorXd l_vecXdImCoherency = l_vecXcdCoherency.imag().cwiseAbs();
			
			uint32_t l_ui32NumValids = 0;
			for (int32 i = 0; i<l_vecXcdCoherency.size()/2; i++) // skip symmetric part
			{

				float64 val = l_vecXdImCoherency(i);
			
				if( (l_vecXdCoherencyDen(i)<l_f64Epsilon)
					|| val!=val) 
				{
					// Set to zero if both channels have very low autospectral power or if the result is NaN 
					val = 0;
				}
				else 
				{
					// Only include valid results in the mean
					l_ui32NumValids++;
				}

				l_pOutputMatrixCoherenceSpectrum->getBuffer()[i*2+0+channel*l_vecXdImCoherency.size()] = val;
				l_pOutputMatrixCoherenceSpectrum->getBuffer()[i*2+1+channel*l_vecXdImCoherency.size()] = val;

				// Compute ImCoherency mean over frequencies
				l_f64MeanCohere += val;

			}

			// Write coherence mean over valid frequencies to output. 
			// The validity test might not matter much for real data that may have power on all bands, but its critical for artificial examples.
			if(l_ui32NumValids) 
			{
				l_pOutputMatrixMeanCoherence->getBuffer()[channel] = l_f64MeanCohere/l_ui32NumValids;
				//cout <<"Imaginary Coherency: " <<l_pOutputMatrixMeanCoherence->getBuffer()[channel] << endl;
			} 
			else 
			{
				l_pOutputMatrixMeanCoherence->getBuffer()[channel] = 0;
			}
			// cout << "Coh " << l_f64MeanCohere << "\n";
		}
		this->activateOutputTrigger(OVP_Algorithm_Connectivity_OutputTriggerId_ProcessDone, true);
	}

	return true;
}
#endif //TARGET_HAS_ThirdPartyEIGEN
